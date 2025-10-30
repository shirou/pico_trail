//! USB Logger test for RP2350
//!
//! This program outputs logs via USB serial to verify basic functionality.
//! No external hardware required - just connect USB and open serial terminal.
//!
//! # Hardware
//!
//! Raspberry Pi Pico 2 W (RP2350)
//!
//! # Usage
//!
//! ```bash
//! ./scripts/build-rp2350.sh --release usb_logger_test
//! # Flash target/usb_logger_test.uf2 to Pico 2 W
//! # Open serial terminal (e.g., screen /dev/ttyACM0 115200)
//! ```

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// Static buffers for USB
static mut USB_CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
static mut USB_BOS_DESCRIPTOR: [u8; 256] = [0; 256];
static mut USB_CONTROL_BUF: [u8; 256] = [0; 256];
static mut USB_SERIAL_STATE: State = State::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Create USB driver
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x16c0, 0x27dd);
    config.manufacturer = Some("pico_trail");
    config.product = Some("USB Logger Test");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for Windows support
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create USB builder with static buffers
    let mut builder = unsafe {
        Builder::new(
            driver,
            config,
            &mut USB_CONFIG_DESCRIPTOR[..],
            &mut USB_BOS_DESCRIPTOR[..],
            &mut [], // msos_descriptor
            &mut USB_CONTROL_BUF[..],
        )
    };

    // Create CDC-ACM class with static state
    let mut class = unsafe { CdcAcmClass::new(&mut builder, &mut USB_SERIAL_STATE, 64) };

    // Build the USB device
    let mut usb = builder.build();

    // Spawn USB task
    spawner.spawn(usb_task(usb)).unwrap();

    // Wait for USB to enumerate
    Timer::after_millis(1000).await;

    let mut counter = 0u32;
    loop {
        let msg = format_args!("Hello from Pico 2 W! Counter: {}\r\n", counter);
        let mut buf = [0u8; 128];
        let len = {
            use core::fmt::Write;
            let mut writer = ArrayWriter::new(&mut buf);
            write!(writer, "{}", msg).ok();
            writer.pos()
        };

        // Try to write (ignore errors if USB not connected)
        let _ = class.write_packet(&buf[..len]).await;

        counter += 1;
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

// Helper to format strings without std
struct ArrayWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> ArrayWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        ArrayWriter { buf, pos: 0 }
    }

    fn pos(&self) -> usize {
        self.pos
    }
}

impl<'a> core::fmt::Write for ArrayWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let len = bytes.len().min(self.buf.len() - self.pos);
        self.buf[self.pos..self.pos + len].copy_from_slice(&bytes[..len]);
        self.pos += len;
        Ok(())
    }
}
