//! Crampon on Anchor in Rust
#![no_std]
#![no_main]
#![allow(dead_code, non_camel_case_types, non_upper_case_globals)]

use panic_halt as _;

use core::{arch::asm, ptr};
use embedded_hal as ehal;
use hal::pac;
pub use stm32l4xx_hal as hal;

use crate::ehal::spi::MODE_3;
use crate::hal::prelude::*;
use crate::hal::spi::Spi;

use cortex_m::asm::delay as cycle_delay;
use cortex_m::interrupt::free;
use cortex_m_rt::entry;

use hal::usb::Peripheral;
use hal::{gpio::*, stm32, stm32::SPI1};
use pac::CorePeripherals;

use anchor::*;

mod adxl345;
mod clock;
mod commands;
mod serialnumber;
mod usb;

const BOOTLOADER_FLAG_ADDR: u32 = 0x2000_6ffc; // 0 and max get clobbered at init
const BOOTLOADER_FLAG_MAGIC: u32 = 0xf026_69ef;
const BOOTLOADER_ST_ADDR: u32 = 0x1fff_0000;

const LED_TOGGLE_TICKS: u32 = CLOCK_FREQ / 8;

type CramponAdxl = adxl345::Adxl<
    Spi<
        SPI1,
        (
            Pin<Alternate<PushPull, 5>, L8, 'B', 3>,
            Pin<Alternate<PushPull, 5>, L8, 'A', 6>,
            Pin<Alternate<PushPull, 5>, L8, 'B', 5>,
        ),
    >,
    Pin<Output<PushPull>, L8, 'A', 4>,
>;

pub struct State {
    config_crc: Option<u32>,
    adxl: CramponAdxl,
}

impl State {
    fn new(adxl: CramponAdxl) -> Self {
        State {
            config_crc: None,
            adxl,
        }
    }

    fn clock(&self) -> &clock::Clock {
        &crampon_global().clock
    }
}

pub struct Crampon {
    core: CorePeripherals,
    pin_led: Pin<Output<PushPull>, H8, 'A', 15>,
    led_tick_time: Option<clock::InstantShort>,
    state: State,
}

pub struct CramponGlobal {
    usb: usb::CramponUsb,
    clock: clock::Clock,
}

struct CramponParts {
    crampon: Crampon,
    global: CramponGlobal,
}

impl Crampon {
    fn init() -> CramponParts {
        let dp = stm32::Peripherals::take().unwrap();
        let mut core = CorePeripherals::take().unwrap();

        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

        let clocks = rcc
            .cfgr
            .hsi48(true)
            .sysclk(80.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        // enable usb clock recovery
        enable_crs();

        // disable Vddusb power isolation
        enable_usb_pwr();

        // Configure the on-board LED
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
        let mut pin_led = gpioa
            .pa15
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        pin_led.set_low(); // Turn on

        let mcu_clock = clock::Clock::init(&mut core.DCB, &mut core.DWT);

        cycle_delay(CLOCK_FREQ / 500); // 2ms delay on usb init
        let usb_peripheral = Peripheral {
            usb: dp.USB,
            pin_dm: gpioa
                .pa11
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
            pin_dp: gpioa
                .pa12
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
        };
        let usb = usb::CramponUsb::init(usb_peripheral);

        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
        let cs = gpioa
            .pa4
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        let sck = gpiob
            .pb3
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let mosi = gpiob
            .pb5
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let miso = gpioa
            .pa6
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
        let spi = Spi::spi1(
            dp.SPI1,
            (sck, miso, mosi),
            MODE_3,
            8.MHz(),
            clocks,
            &mut rcc.apb2,
        );
        let adxl = CramponAdxl::init(spi, cs);

        CramponParts {
            crampon: Crampon {
                core,
                pin_led,
                led_tick_time: None,
                state: State::new(adxl),
            },
            global: CramponGlobal {
                usb,
                clock: mcu_clock,
            },
        }
    }

    fn run_usb<const BUF_SIZE: usize>(&mut self, receive_buffer: &mut FifoBuffer<BUF_SIZE>) {
        // Pump USB read side
        crampon_global().usb.read_into(receive_buffer);
        let recv_data = receive_buffer.data();
        if !recv_data.is_empty() {
            let mut wrap = SliceInputBuffer::new(recv_data);
            KLIPPER_TRANSPORT.receive(&mut wrap, &mut self.state);
            let consumed = recv_data.len() - wrap.available();
            if consumed > 0 {
                receive_buffer.pop(consumed);
            }
        }

        // Pump USB write side
        free(|cs| {
            let mut txbuf = usb::USB_TX_BUFFER.borrow(cs).borrow_mut();
            crampon_global().usb.write_from(cs, &mut txbuf);
        });

        if crampon_global().usb.data_rate() == 1200 {
            unsafe {
                ptr::write(BOOTLOADER_FLAG_ADDR as *mut u32, BOOTLOADER_FLAG_MAGIC);
                asm!("nop"); // write needs time to hit ram
            }
            cortex_m::peripheral::SCB::sys_reset();
        }
    }

    fn run_adxl(&mut self, t: clock::InstantShort) {
        self.state.adxl.run(t);
    }

    fn run_led(&mut self, t: clock::InstantShort) {
        if !self.state.adxl.detected() {
            if let Some(tick) = self.led_tick_time {
                if t.after(tick) {
                    self.pin_led.toggle();
                    self.led_tick_time = Some(tick + LED_TOGGLE_TICKS);
                }
            } else {
                self.led_tick_time = Some(t + LED_TOGGLE_TICKS);
            }
        }
    }
}

/// Enables USB clock recovery for crystal-less operation
fn enable_crs() {
    let rcc = unsafe { &(*stm32::RCC::ptr()) };
    rcc.apb1enr1.modify(|_, w| w.crsen().set_bit());
    let crs = unsafe { &(*stm32::CRS::ptr()) };
    // Initialize clock recovery
    // Set autotrim enabled.
    crs.cr.modify(|_, w| w.autotrimen().set_bit());
    // Enable CR
    crs.cr.modify(|_, w| w.cen().set_bit());
}

/// Enables VddUSB power supply
fn enable_usb_pwr() {
    // Enable PWR peripheral
    let rcc = unsafe { &(*stm32::RCC::ptr()) };
    rcc.apb1enr1.modify(|_, w| w.pwren().set_bit());

    // Enable VddUSB
    let pwr = unsafe { &*stm32::PWR::ptr() };
    pwr.cr2.modify(|_, w| w.usv().set_bit());
}

fn bootloader_check() {
    unsafe {
        // chip needs a reset after a bootloader run to work correctly
        let dirty_reg: u32;
        asm!("mov {0}, r4", out(reg) dirty_reg);
        if dirty_reg != 0 {
            cortex_m::peripheral::SCB::sys_reset();
        }
        if ptr::read(BOOTLOADER_FLAG_ADDR as *const u32) == BOOTLOADER_FLAG_MAGIC {
            ptr::write(BOOTLOADER_FLAG_ADDR as *mut u32, 0);
            let initial_sp = ptr::read(BOOTLOADER_ST_ADDR as *const u32);
            let start_addr = ptr::read((BOOTLOADER_ST_ADDR + 4) as *const u32);
            asm!("mov sp, {0}\nbx {1}", in(reg) initial_sp, in(reg) start_addr);
        }
    }
}

impl CramponParts {
    fn run_forever(mut self) -> ! {
        unsafe {
            CRAMPON_GLOBAL = Some(self.global);
        }

        // We keep the receive buffer here, outside of the State struct.
        // This allows us to later pass the State struct as our context to our commands handlers.
        // If the receive buffer was part of the State struct, we'd have multiple mutable borrows.
        let mut receive_buffer = FifoBuffer::<{ usb::USB_MAX_PACKET_SIZE * 2 }>::new();

        loop {
            crampon_global().usb.poll();
            self.crampon.run_usb(&mut receive_buffer);
            self.crampon.run_adxl(crampon_global().clock.low());
            self.crampon.run_led(crampon_global().clock.low());
        }
    }
}

static mut CRAMPON_GLOBAL: Option<CramponGlobal> = None;

pub fn crampon_global() -> &'static CramponGlobal {
    unsafe { CRAMPON_GLOBAL.as_ref().unwrap() }
}

#[entry]
fn main() -> ! {
    bootloader_check();
    Crampon::init().run_forever();
}

klipper_config_generate!(
    transport = crate::usb::TRANSPORT_OUTPUT: crate::usb::BufferTransportOutput,
    context = &'ctx mut crate::State,
);

#[klipper_constant]
const CLOCK_FREQ: u32 = 80_000_000;

#[klipper_constant]
const MCU: &str = "crampon";

#[klipper_constant]
const STATS_SUMSQ_BASE: u32 = 256;

klipper_enumeration!(
    enum spi_bus {
        spi1,
    }
);

klipper_enumeration!(
    enum pin {
        CS,
    }
);

#[klipper_constant]
const BUS_PINS_spi1: &str = "PA6,PB5,PB3";
