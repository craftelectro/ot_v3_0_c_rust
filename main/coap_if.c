#include "coap_if.h"
#include "logic.h"
#include "config.h"
#include "config_store.h"
#include "rust_payload.h"

#include "esp_openthread_lock.h"
#include "esp_log.h"

#include <openthread/coap.h>
#include <openthread/message.h>
#include <openthread/ip6.h>

#include <string.h>
#include <stdio.h>

#include <openthread/thread.h>


static const char *TAG = "coap_if";
static otInstance *s_ot = NULL;

static otIp6Address s_mcast_all_nodes; // ff03::1

// ---- helpers ----

static void zone_id_str(char *out, size_t n)
{
    const app_config_t *cfg = config_store_get();
    snprintf(out, n, "%d", cfg->zone_id);
}

static otMessage *new_post_msg(void)
{
    otMessage *m = otCoapNewMessage(s_ot, NULL);
    if (!m) return NULL;
    otCoapMessageInit(m, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);
    return m;
}

static void append_uri(otMessage *m, const char *seg)
{
    (void)otCoapMessageAppendUriPathOptions(m, seg);
}

static otMessage *build_state_req_msg(void)
{
    otMessage *m = new_post_msg();
    if (!m) {
        return NULL;
    }

    char zid[8];
    zone_id_str(zid, sizeof(zid));

    append_uri(m, "zone");
    append_uri(m, zid);
    append_uri(m, "state_req");

    otCoapMessageSetPayloadMarker(m);
    otMessageAppend(m, "1", 1);

    return m;
}

static bool rloc16_to_addr(uint16_t rloc16, otIp6Address *addr)
{
    if (!addr || !s_ot) {
        return false;
    }

    const otMeshLocalPrefix *prefix = otThreadGetMeshLocalPrefix(s_ot);
    if (!prefix) {
        return false;
    }

    memset(addr, 0, sizeof(*addr));
    memcpy(addr->mFields.m8, prefix->m8, OT_MESH_LOCAL_PREFIX_SIZE);

    addr->mFields.m8[8] = 0x00;
    addr->mFields.m8[9] = 0x00;
    addr->mFields.m8[10] = 0x00;
    addr->mFields.m8[11] = 0xff;
    addr->mFields.m8[12] = 0xfe;
    addr->mFields.m8[13] = 0x00;
    addr->mFields.m8[14] = (uint8_t)(rloc16 >> 8);
    addr->mFields.m8[15] = (uint8_t)(rloc16 & 0xff);

    return true;
}

static void send_mcast(otMessage *m)
{
    otMessageInfo info;
    memset(&info, 0, sizeof(info));
    info.mPeerAddr = s_mcast_all_nodes;
    info.mPeerPort = OT_DEFAULT_COAP_PORT;

    otError e = otCoapSendRequest(s_ot, m, &info, NULL, NULL);
    if (e != OT_ERROR_NONE) {
        ESP_LOGW(TAG, "otCoapSendRequest(mcast) err=%d", (int)e);
        otMessageFree(m);
    }
}

static void send_ucast(otMessage *m, const otMessageInfo *peer)
{
    otMessageInfo info;
    memset(&info, 0, sizeof(info));
    info.mPeerAddr = peer->mPeerAddr;
    info.mPeerPort = peer->mPeerPort;

    otError e = otCoapSendRequest(s_ot, m, &info, NULL, NULL);
    if (e != OT_ERROR_NONE) {
        ESP_LOGW(TAG, "otCoapSendRequest(ucast) err=%d", (int)e);
        otMessageFree(m);
    }
}

// static void coap_send_empty_ack(otMessage *req, const otMessageInfo *req_info)
// {
//     // ACK нужен только для Confirmable запросов
//     if (otCoapMessageGetType(req) != OT_COAP_TYPE_CONFIRMABLE) {
//         return;
//     }

//     otMessage *ack = otCoapNewMessage(s_ot, NULL);
//     if (!ack) return;

//     otCoapMessageInit(ack, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_EMPTY);
//     otCoapMessageSetMessageId(ack, otCoapMessageGetMessageId(req));

//     // Токен должен совпасть
//     const uint8_t *tok = otCoapMessageGetToken(req);
//     uint8_t tlen = otCoapMessageGetTokenLength(req);
//     if (tlen) {
//         otCoapMessageSetToken(ack, tok, tlen);
//     }

//     // Адрес/порт — как у запроса
//     otCoapSendResponse(s_ot, ack, req_info);
// }

static void coap_send_empty_ack(otMessage *req, const otMessageInfo *req_info)
{
    // ACK нужен только для Confirmable запросов
    if (otCoapMessageGetType(req) != OT_COAP_TYPE_CONFIRMABLE) {
        return;
    }

    otMessage *ack = otCoapNewMessage(s_ot, NULL);
    if (!ack) {
        return;
    }

    // ВАЖНО: InitResponse сам выставляет MessageId/Token по req
    otCoapMessageInitResponse(ack, req, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_EMPTY);

    otError err = otCoapSendResponse(s_ot, ack, req_info);
    if (err != OT_ERROR_NONE) {
        otMessageFree(ack);
    }
}



static bool parse_ip_kv(const char *s, const char *key, otIp6Address *out)
{
    const char *p = strstr(s, key);
    if (!p) return false;
    p += strlen(key);
    if (*p != '=') return false;
    p++;

    // копируем до ';' или конца
    char buf[64];
    size_t i = 0;
    while (*p && *p != ';' && i + 1 < sizeof(buf)) {
        buf[i++] = *p++;
    }
    buf[i] = 0;
    if (i == 0) return false;

    return (otIp6AddressFromString(buf, out) == OT_ERROR_NONE);
}

// static int read_payload(otMessage *msg, char *buf, size_t n)
// {
//     uint16_t off = otMessageGetOffset(msg);
//     uint16_t len = otMessageGetLength(msg);
//     if (len <= off) return 0;
//     uint16_t pl = (uint16_t)(len - off);
//     if (pl >= n) pl = (uint16_t)(n - 1);

//     otMessageRead(msg, off, buf, pl);
//     buf[pl] = 0;
//     return (int)pl;
// }

static int read_payload(otMessage *msg, char *out, size_t out_sz)
{
    if (!out || out_sz == 0) {
        return 0;
    }
    out[0] = 0;

    int len = (int)otMessageGetLength(msg);
    if (len <= 0) {
        return 0;
    }

    uint16_t offset = otMessageGetOffset(msg);
    if (offset >= len) {
        return 0;
    }

    int payload_len = len - (int)offset;
    if (payload_len <= 0) {
        return 0;
    }

    size_t copy = (size_t)payload_len;
    bool truncated = false;
    if (copy > out_sz - 1) {
        copy = out_sz - 1;
        truncated = true;
    }

    int rd = otMessageRead(msg, offset, out, (uint16_t)copy);
    if (rd <= 0) {
        out[0] = 0;
        return 0;
    }

    out[rd] = 0;

    if (truncated) {
        ESP_LOGW(TAG, "payload truncated: len=%d, buf=%zu", payload_len, out_sz - 1);
    }

    return rd;
}


static void send_ok(otMessage *req, const otMessageInfo *info)
{
    otMessage *rsp = otCoapNewMessage(s_ot, NULL);
    if (!rsp) return;

    otCoapMessageInitResponse(rsp, req, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);
    otCoapMessageSetPayloadMarker(rsp);
    otMessageAppend(rsp, "ok", 2);

    otError e = otCoapSendResponse(s_ot, rsp, info);
    if (e != OT_ERROR_NONE) {
        otMessageFree(rsp);
    }
}

// ---- RX handlers ----

// static void on_state_req(void *ctx, otMessage *msg, const otMessageInfo *info)
// {
//     (void)ctx;
//     (void)info;

//     // кто угодно отвечает своим состоянием multicast
//     uint32_t epoch = 0;
//     otIp6Address owner;
//     uint32_t rem_ms = 0;
//     bool active = false;

//     logic_build_state(&epoch, &owner, &rem_ms, &active);
//     coap_if_send_state_rsp(epoch, &owner, rem_ms, active);

//     send_ok(msg, info);
// }

static void on_state_req(void *ctx, otMessage *msg, const otMessageInfo *info)
{
    (void)ctx;

    // ACK только для CON, для NON ничего не отвечаем
    coap_send_empty_ack(msg, info);

    // кто угодно отвечает своим состоянием unicast обратно отправителю
    uint32_t epoch = 0;
    otIp6Address owner;
    uint32_t rem_ms = 0;
    bool active = false;

    logic_build_state(&epoch, &owner, &rem_ms, &active);
    otMessage *rsp = new_post_msg();
    if (rsp) {
        append_uri(rsp, "zone");
        char zid[8];
        zone_id_str(zid, sizeof(zid));
        append_uri(rsp, zid);
        append_uri(rsp, "state_rsp");

        char owner_str[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&owner, owner_str, sizeof(owner_str));

        char pl[200];
        snprintf(pl, sizeof(pl), "e=%lu;a=%u;r=%lu;o=%s",
                 (unsigned long)epoch,
                 active ? 1u : 0u,
                 (unsigned long)rem_ms,
                 owner_str);

        otCoapMessageSetPayloadMarker(rsp);
        otMessageAppend(rsp, pl, (uint16_t)strlen(pl));
        send_ucast(rsp, info);
    }

    // НЕ send_ok() !
}


static void on_state_rsp(void *ctx, otMessage *msg, const otMessageInfo *info)
{
    (void)ctx;

    otIp6Address my;
    if (coap_if_get_my_meshlocal_eid(&my)) {
        if (memcmp(&my, &info->mPeerAddr, sizeof(my)) == 0) {
            ESP_LOGD(TAG, "ignore state_rsp from self");
            return;
        }
    }


    (void)info;

    char buf[160];
    int len = read_payload(msg, buf, sizeof(buf));

    // формат: e=123;a=1;r=600000;o=fdde:....
    rust_parsed_t parsed = {0};
    if (!rust_parse_payload((const uint8_t *)buf, (uint32_t)len, &parsed)) {
        return;
    }
    if (!parsed.has_epoch) return;
    if (!parsed.has_active) return;

    otIp6Address owner;
    if (!parse_ip_kv(buf, "o", &owner)) memset(&owner, 0, sizeof(owner));

    if (!logic_post_parsed(LOGIC_PARSED_STATE_RSP, &parsed, &owner, true)) {
        return;
    }

    send_ok(msg, info);
}

static void on_trigger(void *ctx, otMessage *msg, const otMessageInfo *info)
{
    (void)ctx;

    char buf[96];
    int len = read_payload(msg, buf, sizeof(buf));

    rust_parsed_t parsed = {0};
    if (!rust_parse_payload((const uint8_t *)buf, (uint32_t)len, &parsed)) {
        return;
    }
    if (!logic_post_parsed(LOGIC_PARSED_TRIGGER, &parsed, &info->mPeerAddr, true)) {
        return;
    }

    // ACK только для CON, для NON ничего не отвечаем
    coap_send_empty_ack(msg, info);

    if (parsed.has_epoch) {
        uint32_t rem_ms = parsed.has_rem_ms ? parsed.rem_ms : config_store_get()->auto_hold_ms;
        ESP_LOGI(TAG, "RX trigger from peer, epoch=%lu rem_ms=%lu",
                 (unsigned long)parsed.epoch, (unsigned long)rem_ms);
    }

    // logic_post_state_response(epoch, &owner, rem_ms, active);


    // НЕ делать send_ok() здесь!
}


static void on_off(void *ctx, otMessage *msg, const otMessageInfo *info)
{
    (void)ctx;

    char buf[64];
    int len = read_payload(msg, buf, sizeof(buf));

    rust_parsed_t parsed = {0};
    if (!rust_parse_payload((const uint8_t *)buf, (uint32_t)len, &parsed)) {
        return;
    }
    if (!logic_post_parsed(LOGIC_PARSED_OFF, &parsed, NULL, true)) {
        return;
    }

    if (parsed.has_epoch) {
        ESP_LOGI(TAG, "RX off epoch=%lu", (unsigned long)parsed.epoch);
    }

    send_ok(msg, info);
}

// ---- RX: mode ----
// payload варианты:
//  - m=0;          (global OFF)
//  - m=2;          (global AUTO)
//  - m=0;z=3;      (zone 3 OFF)
//  - clr=1;        (clear global override -> return to local)
//  - clr=2;z=3;    (clear zone override for zone 3)
//  - clr=3;        (clear node override; по факту unicast)

static void on_mode_set(void *ctx, otMessage *msg, const otMessageInfo *info)
{
    (void)ctx;

    char buf[128];
    int len = read_payload(msg, buf, sizeof(buf));

    ESP_LOGI(TAG, "RX /mode from %x.. payload='%s' sock0=%02x",
         info->mPeerAddr.mFields.m8[15], buf, info->mSockAddr.mFields.m8[0]);

    rust_parsed_t parsed = {0};
    if (!rust_parse_payload((const uint8_t *)buf, (uint32_t)len, &parsed)) {
        return;
    }

    // Определяем multicast по адресу, на который прилетело (local sockaddr)
    // Для multicast IPv6 первый байт = 0xFF.
    // bool is_multicast = (info->mSockAddr.mAddress.mFields.m8[0] == 0xFF);
    bool is_multicast = (info->mSockAddr.mFields.m8[0] == 0xFF);

    if (!logic_post_parsed(LOGIC_PARSED_MODE, &parsed, NULL, is_multicast)) {
        return;
    }

    send_ok(msg, info);
}


// ---- register ----

void coap_if_register(otInstance *ot)
{
    esp_openthread_lock_acquire(portMAX_DELAY);
    s_ot = ot;

    otIp6AddressFromString("ff03::1", &s_mcast_all_nodes);
    otCoapStart(s_ot, OT_DEFAULT_COAP_PORT);

    static char path_state_req[40];
    static char path_state_rsp[40];
    static char path_trigger[40];
    static char path_off[40];
    static char path_mode[40];


    uint8_t zid = config_store_get()->zone_id;
    snprintf(path_state_req, sizeof(path_state_req), "zone/%d/state_req", zid);
    snprintf(path_state_rsp, sizeof(path_state_rsp), "zone/%d/state_rsp", zid);
    snprintf(path_trigger,   sizeof(path_trigger),   "zone/%d/trigger",   zid);
    snprintf(path_off,       sizeof(path_off),       "zone/%d/off",       zid);
    snprintf(path_mode,      sizeof(path_mode),      "zone/%d/mode",      zid);


    static otCoapResource r_state_req;
    static otCoapResource r_state_rsp;
    static otCoapResource r_trigger;
    static otCoapResource r_off;
    static otCoapResource r_mode;


    memset(&r_state_req, 0, sizeof(r_state_req));
    memset(&r_state_rsp, 0, sizeof(r_state_rsp));
    memset(&r_trigger,   0, sizeof(r_trigger));
    memset(&r_off,       0, sizeof(r_off));
    memset(&r_mode, 0, sizeof(r_mode));


    r_state_req.mUriPath = path_state_req;
    r_state_req.mHandler = on_state_req;

    r_state_rsp.mUriPath = path_state_rsp;
    r_state_rsp.mHandler = on_state_rsp;

    r_trigger.mUriPath = path_trigger;
    r_trigger.mHandler = on_trigger;

    r_off.mUriPath = path_off;
    r_off.mHandler = on_off;

    r_mode.mUriPath = path_mode;
    r_mode.mHandler = on_mode_set;


    otCoapAddResource(s_ot, &r_state_req);
    otCoapAddResource(s_ot, &r_state_rsp);
    otCoapAddResource(s_ot, &r_trigger);
    otCoapAddResource(s_ot, &r_off);
    otCoapAddResource(s_ot, &r_mode);


    esp_openthread_lock_release();

    ESP_LOGI(TAG, "CoAP: /%s /%s /%s /%s /%s",
         path_state_req, path_state_rsp, path_trigger, path_off, path_mode);


    // ESP_LOGI(TAG, "CoAP: /%s /%s /%s /%s",
    //          path_state_req, path_state_rsp, path_trigger, path_off);
}

// ---- SEND (multicast) ----

void coap_if_send_state_req(void)
{
    if (!s_ot) return;

    uint16_t leader_rloc = otThreadGetLeaderRloc(s_ot);
    if (leader_rloc != 0 && leader_rloc != 0xfffe && leader_rloc != 0xffff) {
        otIp6Address leader_addr;
        if (rloc16_to_addr(leader_rloc, &leader_addr)) {
            otMessage *ucast = build_state_req_msg();
            if (ucast) {
                otMessageInfo info;
                memset(&info, 0, sizeof(info));
                info.mPeerAddr = leader_addr;
                info.mPeerPort = OT_DEFAULT_COAP_PORT;
                send_ucast(ucast, &info);
            }
        }
    }

    otMessage *mcast = build_state_req_msg();
    if (mcast) {
        send_mcast(mcast);
    }
}

void coap_if_send_state_rsp(uint32_t epoch,
                            const otIp6Address *owner,
                            uint32_t remaining_ms,
                            bool active)
{
    if (!s_ot) return;

    otMessage *m = new_post_msg();
    if (!m) return;

    char zid[8];
    zone_id_str(zid, sizeof(zid));

    append_uri(m, "zone");
    append_uri(m, zid);
    append_uri(m, "state_rsp");

    char owner_str[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(owner, owner_str, sizeof(owner_str));

    char pl[200];
    // e=..;a=..;r=..;o=....
    snprintf(pl, sizeof(pl), "e=%lu;a=%u;r=%lu;o=%s",
             (unsigned long)epoch,
             active ? 1u : 0u,
             (unsigned long)remaining_ms,
             owner_str);

    otCoapMessageSetPayloadMarker(m);
    otMessageAppend(m, pl, (uint16_t)strlen(pl));

    send_mcast(m);
}

// void coap_if_send_trigger(uint32_t epoch, uint32_t hold_ms)
// {
//     if (!s_ot) return;

//     otMessage *m = new_post_msg();
//     if (!m) return;

//     char zid[8];
//     zone_id_str(zid, sizeof(zid));

//     append_uri(m, "zone");
//     append_uri(m, zid);
//     append_uri(m, "trigger");

//     char pl[80];
//     snprintf(pl, sizeof(pl), "e=%lu;h=%lu",
//              (unsigned long)epoch, (unsigned long)hold_ms);

//     otCoapMessageSetPayloadMarker(m);
//     otMessageAppend(m, pl, (uint16_t)strlen(pl));

//     send_mcast(m);
// }

void coap_if_send_trigger(uint32_t epoch, uint32_t rem_ms)
{
    if (!coap_if_thread_ready()) {
        ESP_LOGW(TAG, "TX trigger skipped: thread not ready");
        return;
    }

    if (!s_ot) return;

    otMessage *m = new_post_msg();
    if (!m) return;

    char zid[8];
    zone_id_str(zid, sizeof(zid));

    append_uri(m, "zone");
    append_uri(m, zid);
    append_uri(m, "trigger");

    char pl[80];
    // новый формат
    snprintf(pl, sizeof(pl), "epoch=%lu&rem_ms=%lu",
             (unsigned long)epoch, (unsigned long)rem_ms);

    otCoapMessageSetPayloadMarker(m);
    otMessageAppend(m, pl, (uint16_t)strlen(pl));

    send_mcast(m);
}


void coap_if_send_off(uint32_t epoch)
{
    if (!s_ot) return;

    otMessage *m = new_post_msg();
    if (!m) return;

    char zid[8];
    zone_id_str(zid, sizeof(zid));

    append_uri(m, "zone");
    append_uri(m, zid);
    append_uri(m, "off");

    char pl[48];
    snprintf(pl, sizeof(pl), "e=%lu", (unsigned long)epoch);

    otCoapMessageSetPayloadMarker(m);
    otMessageAppend(m, pl, (uint16_t)strlen(pl));

    send_mcast(m);
}

bool coap_if_get_my_meshlocal_eid(otIp6Address *out)
{
    if (!s_ot || !out) return false;
    const otNetifAddress *a = otIp6GetUnicastAddresses(s_ot);
    while (a) {
        // берём первый mesh-local (обычно начинается с fd..)
        *out = a->mAddress;
        return true;
    }
    return false;
}


bool coap_if_thread_ready(void)
{
    if (!s_ot) return false;
    otDeviceRole r = otThreadGetDeviceRole(s_ot);
    return (r != OT_DEVICE_ROLE_DISABLED && r != OT_DEVICE_ROLE_DETACHED);
}
