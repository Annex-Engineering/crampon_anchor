use crate::hal;

pub fn get_serial() -> &'static str {
    static mut SERIALNUMBER: [u8; 32] = [0; 32];

    fn hexchar(n: u8) -> u8 {
        if n >= 0xA {
            n + 0x41 - 0xA
        } else {
            n + 0x30
        }
    }

    unsafe {
        if SERIALNUMBER[0] == 0 {
            for (i, b) in hal::signature::Uid::as_bytes().into_iter().enumerate() {
                SERIALNUMBER[i * 2] = hexchar((b >> 4) & 0xF);
                SERIALNUMBER[i * 2 + 1] = hexchar(b & 0xF);
            }
        }
        core::str::from_utf8_unchecked(&SERIALNUMBER)
    }
}
