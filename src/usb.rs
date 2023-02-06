use crate::hal::{usb::Peripheral, usb::UsbBus};
use anchor::*;
use core::cell::RefCell;
use cortex_m::interrupt::{free, CriticalSection, Mutex};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{CdcAcmClass, USB_CLASS_CDC};

pub const USB_MAX_PACKET_SIZE: usize = 64;

struct CramponUsbDevice {
    bus: UsbDevice<'static, UsbBus<Peripheral>>,
    serial: CdcAcmClass<'static, UsbBus<Peripheral>>,
    full_count: u8,
}

impl CramponUsbDevice {
    pub fn poll(&mut self) {
        self.bus.poll(&mut [&mut self.serial]);
    }

    pub fn read_into<const BUF_SIZE: usize>(&mut self, buffer: &mut FifoBuffer<BUF_SIZE>) {
        while let Ok(n) = self.serial.read_packet(buffer.receive_buffer()) {
            buffer.advance(n);
        }
    }

    pub fn write_from<const BUF_SIZE: usize>(&mut self, buffer: &mut FifoBuffer<BUF_SIZE>) {
        if buffer.is_empty() && self.full_count == 0 {
            // Fast path: nothing to do
            return;
        }
        let max_packet_size = self.serial.max_packet_size();
        let data = buffer.data();
        let len = data.len().clamp(0, max_packet_size as usize) as u16;
        let data = &data[..(len as usize)];

        let (consumed, write) = if len == max_packet_size && self.full_count > 10 {
            // Write one byte less
            (len - 1, &data[..(len - 1) as usize])
        } else if len == 0 {
            // Write zero length packet
            (0u16, &[] as &[u8])
        } else {
            // Normal write
            (len, data)
        };

        match self.serial.write_packet(write) {
            Ok(0) => {
                self.full_count = 0;
            }
            Ok(n) => {
                if (n as u16) < max_packet_size {
                    self.full_count = 0;
                }
                buffer.pop(n)
            }
            Err(UsbError::WouldBlock) => {} // Don't consume from the input buffer
            Err(_) => buffer.pop(consumed as usize), // Ignore errors but consume the data
        }
    }

    pub fn data_rate(&mut self) -> u32 {
        self.serial.line_coding().data_rate()
    }
}

pub struct CramponUsb(Mutex<RefCell<CramponUsbDevice>>);

impl CramponUsb {
    pub fn init(peripheral: Peripheral) -> CramponUsb {
        let allocator = unsafe {
            static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;
            if USB_ALLOCATOR.is_none() {
                USB_ALLOCATOR = Some(UsbBus::new(peripheral));
            }
            USB_ALLOCATOR.as_ref().unwrap()
        };

        let serial = CdcAcmClass::new(allocator, USB_MAX_PACKET_SIZE as u16);
        let bus = UsbDeviceBuilder::new(allocator, UsbVidPid(0x0483, 0xaeca))
            .composite_with_iads()
            .manufacturer("Annex Engineering")
            .product("Crampon")
            .serial_number(crate::serialnumber::get_serial())
            .device_class(USB_CLASS_CDC)
            .build();

        CramponUsb(Mutex::new(RefCell::new(CramponUsbDevice {
            bus,
            serial,
            full_count: 0,
        })))
    }

    pub fn poll(&self) {
        free(|cs| self.0.borrow(cs).borrow_mut().poll());
    }

    pub fn read_into<const BUF_SIZE: usize>(&self, buffer: &mut FifoBuffer<BUF_SIZE>) {
        free(|cs| self.0.borrow(cs).borrow_mut().read_into(buffer));
    }

    pub fn write_from<const BUF_SIZE: usize>(
        &self,
        cs: &CriticalSection,
        buffer: &mut FifoBuffer<BUF_SIZE>,
    ) {
        self.0.borrow(cs).borrow_mut().write_from(buffer);
    }

    pub fn data_rate(&self) -> u32 {
        free(|cs| self.0.borrow(cs).borrow_mut().data_rate())
    }
}

pub static USB_TX_BUFFER: Mutex<RefCell<FifoBuffer<{ USB_MAX_PACKET_SIZE * 2 }>>> =
    Mutex::new(RefCell::new(FifoBuffer::new()));
pub(crate) struct BufferTransportOutput;

impl TransportOutput for BufferTransportOutput {
    type Output = ScratchOutput;
    fn output(&self, f: impl FnOnce(&mut Self::Output)) {
        let mut scratch = ScratchOutput::new();
        f(&mut scratch);
        let output = scratch.result();
        free(|cs| USB_TX_BUFFER.borrow(cs).borrow_mut().extend(output));
    }
}

pub(crate) const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;
