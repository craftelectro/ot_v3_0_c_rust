#![no_std]

use core::panic::PanicInfo;

#[no_mangle]
pub extern "C" fn rust_parse_payload() -> u32 {
    1
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
