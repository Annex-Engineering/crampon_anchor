# Crampon Anchor Firmware

This project is a firmware for the Annex Engineering [Crampon](https://github.com/Annex-Engineering/Annex_Engineering_PCBs/tree/master/crampon), an accelerometer resonance measuring device, written in Rust using the [Anchor](https://github.com/Annex-Engineering/anchor) library to speak with Klipper.

It demonstrates how firmware can be rapidly developed for new sensors or new
MCU targets using Anchor.

## Loading Release Builds

Crampon is a single device, and Rust firmware is more easily built on a desktop
class PC using cross compilers, so a pre-built binary is provided for ease of use.

Checkout the repository and load your device using:
```
% git checkout https://github.com/Annex-Engineering/crampon_anchor.git
% ./crampon_anchor/update.sh
```

After the update completes, the device should be available as:
`/dev/serial/by-id/usb-Annex_Engineering_Crampon-if00`.

Devices running an older build may have a serial number in the path.

## Klipper Config

```ini
[mcu crampon]
serial: /dev/serial/by-id/usb-Annex_Engineering_Crampon-if00

[adxl345]
cs_pin: crampon:CS

[resonance_tester]
accel_chip: adxl345
probe_points: 90, 90, 20
```

## Developers

---

## Building Firmware

To compile the project, you will need a Rust toolchain installed, `cargo-binutils`, and the compile target for ARM Cortex-M4F. They can be installed with:

```
% rustup component add llvm-tools-preview
% rustup target add thumbv7em-none-eabihf
```

To build the project, and convert the executable to a `.bin` file, run:
```
% cargo build --release
% rust-objcopy target/thumbv7em-none-eabihf/release/crampon_anchor -O binary crampon.bin
```

To enter the bootloader and program the device, run:
```
% stty -F /dev/serial/by-id/usb-<DEVICEPATH> 1200
% sudo dfu-util -d ,0483:df11 -a 0 -D crampon.bin --dfuse-address 0x08000000:leave
```

After the update completes, the device should be available as:
`/dev/serial/by-id/usb-Annex_Engineering_Crampon-if00`.

To enable the serial number in the device path, simply enable the `serialnumber` cargo feature using the following command to build:
```
% cargo build --release --features serialnumber
```