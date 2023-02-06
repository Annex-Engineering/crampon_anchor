use crate::State;
use anchor::*;
use core::cell::Cell;
use cortex_m::peripheral::{DCB, DWT};

pub struct Clock {
    high: Cell<u32>,
    last_low: Cell<u32>,
}

impl Clock {
    pub fn init(dcb: &mut DCB, dwt: &mut DWT) -> Clock {
        dcb.enable_trace();
        dwt.enable_cycle_counter();
        Clock {
            high: Cell::new(0),
            last_low: Cell::new(0),
        }
    }

    pub fn low(&self) -> InstantShort {
        InstantShort(DWT::cycle_count())
    }

    pub fn update_high(&self) {
        let low = self.low().0;
        if low < self.last_low.get() {
            self.high.set(self.high.get() + 1);
        }
        self.last_low.set(low);
    }

    pub fn full(&self) -> InstantFull {
        self.update_high();
        InstantFull(((self.high.get() as u64) << 32) | (self.low().0 as u64))
    }
}

#[derive(Copy, Clone)]
pub struct InstantShort(u32);

impl InstantShort {
    pub fn new(t: u32) -> InstantShort {
        InstantShort(t)
    }

    pub fn after(&self, other: impl AsRef<Self>) -> bool {
        other.as_ref().0.wrapping_sub(self.0) & 0x8000_0000 != 0
    }
}

impl core::ops::AddAssign<u32> for InstantShort {
    fn add_assign(&mut self, rhs: u32) {
        self.0 = self.0.wrapping_add(rhs);
    }
}

impl core::ops::Add<u32> for InstantShort {
    type Output = Self;
    fn add(self, rhs: u32) -> Self::Output {
        InstantShort(self.0.wrapping_add(rhs))
    }
}

impl core::convert::AsRef<InstantShort> for InstantShort {
    fn as_ref(&self) -> &InstantShort {
        self
    }
}

impl From<InstantShort> for u32 {
    fn from(t: InstantShort) -> Self {
        t.0
    }
}

#[derive(Copy, Clone)]
pub struct InstantFull(u64);

impl From<InstantFull> for u64 {
    fn from(t: InstantFull) -> Self {
        t.0
    }
}

#[klipper_command]
pub fn get_uptime(context: &mut State) {
    let c: u64 = context.clock().full().into();
    klipper_reply!(
        uptime,
        high: u32 = (c >> 32) as u32,
        clock: u32 = (c & 0xFFFFFFFF) as u32
    );
}

#[klipper_command]
pub fn get_clock(context: &State) {
    klipper_reply!(clock, clock: u32 = context.clock().low().into());
}
