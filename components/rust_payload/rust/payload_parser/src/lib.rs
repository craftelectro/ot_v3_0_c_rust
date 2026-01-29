#![no_std]

use core::panic::PanicInfo;

#[repr(C)]
#[derive(Copy, Clone)]
pub struct RustParsed {
    pub has_epoch: u8,
    pub has_rem_ms: u8,
    pub has_active: u8,
    pub has_mode: u8,
    pub has_clr: u8,
    pub has_z: u8,
    pub has_m: u8,
    pub _reserved: u8,
    pub epoch: u32,
    pub rem_ms: u32,
    pub active: u32,
    pub mode: u32,
    pub clr: u32,
    pub z: u32,
    pub m: u32,
}

impl Default for RustParsed {
    fn default() -> Self {
        RustParsed {
            has_epoch: 0,
            has_rem_ms: 0,
            has_active: 0,
            has_mode: 0,
            has_clr: 0,
            has_z: 0,
            has_m: 0,
            _reserved: 0,
            epoch: 0,
            rem_ms: 0,
            active: 0,
            mode: 0,
            clr: 0,
            z: 0,
            m: 0,
        }
    }
}

#[no_mangle]
pub extern "C" fn rust_parse_payload(buf: *const u8, len: u32, out: *mut RustParsed) -> u32 {
    if buf.is_null() || out.is_null() {
        return 0;
    }

    let bytes = unsafe { core::slice::from_raw_parts(buf, len as usize) };
    let parsed = match parse_payload(bytes) {
        Ok(parsed) => parsed,
        Err(()) => return 0,
    };

    unsafe {
        *out = parsed;
    }
    1
}

fn parse_payload(bytes: &[u8]) -> Result<RustParsed, ()> {
    let mut out = RustParsed::default();
    let mut idx = 0;

    while idx < bytes.len() {
        skip_spaces(bytes, &mut idx);
        if idx >= bytes.len() {
            break;
        }
        if is_sep(bytes[idx]) {
            idx += 1;
            continue;
        }

        let key_start = idx;
        while idx < bytes.len() && bytes[idx] != b'=' && !is_sep(bytes[idx]) {
            idx += 1;
        }
        let key_end = idx;
        if idx >= bytes.len() || bytes[idx] != b'=' {
            skip_until_sep(bytes, &mut idx);
            continue;
        }
        idx += 1;

        skip_spaces(bytes, &mut idx);
        let val_start = idx;
        while idx < bytes.len() && !is_sep(bytes[idx]) {
            idx += 1;
        }
        let val_end = idx;

        let key = trim_spaces(&bytes[key_start..key_end]);
        let val = trim_spaces(&bytes[val_start..val_end]);

        if !key.is_empty() && !val.is_empty() {
            let value = parse_u32(val).ok_or(())?;
            match key {
                b"epoch" => {
                    out.has_epoch = 1;
                    out.epoch = value;
                }
                b"rem_ms" => {
                    out.has_rem_ms = 1;
                    out.rem_ms = value;
                }
                b"active" => {
                    if value > 1 {
                        return Err(());
                    }
                    out.has_active = 1;
                    out.active = value;
                }
                b"mode" => {
                    if value > u8::MAX as u32 {
                        return Err(());
                    }
                    out.has_mode = 1;
                    out.mode = value;
                }
                b"clr" => {
                    out.has_clr = 1;
                    out.clr = value;
                }
                b"z" => {
                    out.has_z = 1;
                    out.z = value;
                }
                b"m" => {
                    out.has_m = 1;
                    out.m = value;
                }
                b"e" => {
                    out.has_epoch = 1;
                    out.epoch = value;
                }
                b"h" => {
                    out.has_rem_ms = 1;
                    out.rem_ms = value;
                }
                b"a" => {
                    if value > 1 {
                        return Err(());
                    }
                    out.has_active = 1;
                    out.active = value;
                }
                _ => {}
            }
        }
    }

    Ok(out)
}

fn parse_u32(bytes: &[u8]) -> Option<u32> {
    let mut value: u32 = 0;
    let mut seen = false;
    for &b in bytes {
        if b < b'0' || b > b'9' {
            return None;
        }
        seen = true;
        let digit = (b - b'0') as u32;
        value = value.checked_mul(10)?;
        value = value.checked_add(digit)?;
    }
    if seen { Some(value) } else { None }
}

fn is_sep(b: u8) -> bool {
    b == b';' || b == b'&'
}

fn skip_spaces(bytes: &[u8], idx: &mut usize) {
    while *idx < bytes.len() && bytes[*idx].is_ascii_whitespace() {
        *idx += 1;
    }
}

fn skip_until_sep(bytes: &[u8], idx: &mut usize) {
    while *idx < bytes.len() && !is_sep(bytes[*idx]) {
        *idx += 1;
    }
}

fn trim_spaces(bytes: &[u8]) -> &[u8] {
    let mut start = 0;
    let mut end = bytes.len();
    while start < end && bytes[start].is_ascii_whitespace() {
        start += 1;
    }
    while end > start && bytes[end - 1].is_ascii_whitespace() {
        end -= 1;
    }
    &bytes[start..end]
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::{parse_payload, RustParsed};

    #[test]
    fn parses_basic_fields() {
        let out = parse_payload(b"epoch=10;rem_ms=250;active=1;mode=2").unwrap();
        assert_eq!(out.has_epoch, 1);
        assert_eq!(out.epoch, 10);
        assert_eq!(out.has_rem_ms, 1);
        assert_eq!(out.rem_ms, 250);
        assert_eq!(out.has_active, 1);
        assert_eq!(out.active, 1);
        assert_eq!(out.has_mode, 1);
        assert_eq!(out.mode, 2);
    }

    #[test]
    fn parses_legacy_keys_and_spaces() {
        let out = parse_payload(b" e = 7 & h = 42 ; a = 0 ").unwrap();
        assert_eq!(out.has_epoch, 1);
        assert_eq!(out.epoch, 7);
        assert_eq!(out.has_rem_ms, 1);
        assert_eq!(out.rem_ms, 42);
        assert_eq!(out.has_active, 1);
        assert_eq!(out.active, 0);
    }

    #[test]
    fn ignores_unknown_keys() {
        let out = parse_payload(b"foo=1;epoch=3").unwrap();
        assert_eq!(out.has_epoch, 1);
        assert_eq!(out.epoch, 3);
    }

    #[test]
    fn rejects_overflow() {
        let res = parse_payload(b"epoch=999999999999");
        assert!(res.is_err());
    }

    #[test]
    fn rejects_non_decimal() {
        let res = parse_payload(b"epoch=12x");
        assert!(res.is_err());
    }

    #[test]
    fn rejects_active_out_of_range() {
        let res = parse_payload(b"active=2");
        assert!(res.is_err());
    }

    #[test]
    fn parses_mode_aliases() {
        let out = parse_payload(b"m=1;clr=2;z=3").unwrap();
        assert_eq!(out.has_m, 1);
        assert_eq!(out.m, 1);
        assert_eq!(out.has_clr, 1);
        assert_eq!(out.clr, 2);
        assert_eq!(out.has_z, 1);
        assert_eq!(out.z, 3);
    }
}
