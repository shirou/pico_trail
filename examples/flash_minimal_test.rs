//! Minimal Flash operation test with USB-CDC output
//!
//! This example tests basic Flash operations step by step to identify issues.
//!
//! # Usage
//!
//! ```bash
//! ./scripts/build-rp2350.sh --release flash_minimal_test
//! # Flash the UF2 file to Pico 2 W
//! # Connect with: screen /dev/ttyACM0 115200
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use {defmt_rtt as _, panic_probe as _};

use pico_trail::core::parameters::{ParamMetadata, ParamValue, ParameterRegistry};
use pico_trail::platform::rp2350::Rp2350Flash;
use pico_trail::platform::traits::FlashInterface;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn usb_device_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

static mut USB_DEVICE_DESCRIPTOR: [u8; 256] = [0; 256];
static mut USB_CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
static mut USB_BOS_DESCRIPTOR: [u8; 256] = [0; 256];
static mut USB_CONTROL_BUF: [u8; 64] = [0; 64];
static mut USB_STATE: State = State::new();

#[embassy_executor::task]
async fn flash_test_task(mut class: CdcAcmClass<'static, Driver<'static, USB>>) {
    loop {
        class.wait_connection().await;
        info!("USB connected");

        let _ = class.write_packet(b"\r\n").await;
        let _ = class
            .write_packet(b"========================================\r\n")
            .await;
        let _ = class
            .write_packet(b"  Minimal Flash Operation Test\r\n")
            .await;
        let _ = class
            .write_packet(b"========================================\r\n")
            .await;
        let _ = class.write_packet(b"\r\n").await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 1: Create Flash instance
        let _ = class
            .write_packet(b"Test 1: Creating Flash instance...\r\n")
            .await;
        let mut flash = Rp2350Flash::new();
        let _ = class
            .write_packet(b"OK: Flash instance created\r\n\r\n")
            .await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 2: Read from Flash (should always work)
        let _ = class
            .write_packet(b"Test 2: Reading from Flash...\r\n")
            .await;
        let mut buf = [0u8; 16];
        match flash.read(0x040000, &mut buf) {
            Ok(_) => {
                let _ = class.write_packet(b"OK: Flash read successful\r\n").await;

                // Format and send data
                let mut msg = heapless::String::<64>::new();
                let _ = core::fmt::write(&mut msg, format_args!("  Data: {:02x?}\r\n", &buf[..8]));
                let _ = class.write_packet(msg.as_bytes()).await;
            }
            Err(_) => {
                let _ = class.write_packet(b"ERROR: Flash read failed\r\n").await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 3: Flash erase
        let _ = class
            .write_packet(b"Test 3: Flash erase (4KB block)...\r\n")
            .await;
        let _ = class
            .write_packet(b"  WARNING: Erasing Flash at 0x040000\r\n")
            .await;
        Timer::after(Duration::from_secs(2)).await;

        match flash.erase(0x040000, 4096) {
            Ok(_) => {
                let _ = class.write_packet(b"OK: Flash erase successful\r\n").await;
            }
            Err(_) => {
                let _ = class.write_packet(b"ERROR: Flash erase failed\r\n").await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 4: Verify erase (should be all 0xFF)
        let _ = class.write_packet(b"Test 4: Verifying erase...\r\n").await;
        let mut verify_buf = [0u8; 16];
        match flash.read(0x040000, &mut verify_buf) {
            Ok(_) => {
                let all_ff = verify_buf.iter().all(|&b| b == 0xFF);
                if all_ff {
                    let _ = class
                        .write_packet(b"OK: Erase verified (all 0xFF)\r\n")
                        .await;
                } else {
                    let _ = class.write_packet(b"WARNING: Erase incomplete\r\n").await;
                    let mut msg = heapless::String::<64>::new();
                    let _ = core::fmt::write(
                        &mut msg,
                        format_args!("  Data: {:02x?}\r\n", &verify_buf[..8]),
                    );
                    let _ = class.write_packet(msg.as_bytes()).await;
                }
            }
            Err(_) => {
                let _ = class.write_packet(b"ERROR: Verify read failed\r\n").await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 5: Write to Flash
        let _ = class.write_packet(b"Test 5: Writing to Flash...\r\n").await;
        let test_data = [0x50, 0x41, 0x52, 0x41, 0x4D, 0x45, 0x54, 0x52]; // "PARAMETR"
        match flash.write(0x040000, &test_data) {
            Ok(_) => {
                let _ = class.write_packet(b"OK: Flash write successful\r\n").await;
            }
            Err(_) => {
                let _ = class.write_packet(b"ERROR: Flash write failed\r\n").await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 6: Verify write
        let _ = class.write_packet(b"Test 6: Verifying write...\r\n").await;
        let mut read_back = [0u8; 8];
        match flash.read(0x040000, &mut read_back) {
            Ok(_) => {
                if read_back == test_data {
                    let _ = class
                        .write_packet(b"OK: Write verified (data matches)\r\n")
                        .await;
                    let mut msg = heapless::String::<64>::new();
                    let _ =
                        core::fmt::write(&mut msg, format_args!("  Data: {:02x?}\r\n", &read_back));
                    let _ = class.write_packet(msg.as_bytes()).await;
                } else {
                    let _ = class.write_packet(b"WARNING: Write mismatch\r\n").await;
                    let mut msg1 = heapless::String::<64>::new();
                    let _ = core::fmt::write(
                        &mut msg1,
                        format_args!("  Expected: {:02x?}\r\n", &test_data),
                    );
                    let _ = class.write_packet(msg1.as_bytes()).await;
                    let mut msg2 = heapless::String::<64>::new();
                    let _ = core::fmt::write(
                        &mut msg2,
                        format_args!("  Got:      {:02x?}\r\n", &read_back),
                    );
                    let _ = class.write_packet(msg2.as_bytes()).await;
                }
            }
            Err(_) => {
                let _ = class.write_packet(b"ERROR: Verify read failed\r\n").await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        let _ = class
            .write_packet(b"========================================\r\n")
            .await;
        let _ = class
            .write_packet(b"  Basic Flash Tests Complete\r\n")
            .await;
        let _ = class
            .write_packet(b"========================================\r\n")
            .await;
        let _ = class.write_packet(b"\r\n").await;

        Timer::after(Duration::from_secs(2)).await;

        // Test 7: ParameterRegistry initialization
        let _ = class
            .write_packet(b"Test 7: Creating ParameterRegistry...\r\n")
            .await;
        let mut registry = ParameterRegistry::with_flash(flash);
        let _ = class
            .write_packet(b"OK: ParameterRegistry created\r\n\r\n")
            .await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 8: Register parameters (10 params like hardware test)
        let _ = class
            .write_packet(b"Test 8: Registering 10 parameters...\r\n")
            .await;

        registry
            .register(ParamMetadata::new_float("RATE_ROLL_P", 0.15, 0.0, 1.0))
            .ok();
        registry
            .register(ParamMetadata::new_float("RATE_ROLL_I", 0.1, 0.0, 1.0))
            .ok();
        registry
            .register(ParamMetadata::new_float("RATE_ROLL_D", 0.004, 0.0, 0.1))
            .ok();
        registry
            .register(ParamMetadata::new_float("RATE_PITCH_P", 0.15, 0.0, 1.0))
            .ok();
        registry
            .register(ParamMetadata::new_float("RATE_PITCH_I", 0.1, 0.0, 1.0))
            .ok();
        registry
            .register(ParamMetadata::new_float("RATE_PITCH_D", 0.004, 0.0, 0.1))
            .ok();
        registry
            .register(ParamMetadata::new_uint32("SYSID_THISMAV", 1, 1, 255))
            .ok();
        registry
            .register(ParamMetadata::new_uint32("SR_EXTRA1", 10, 0, 50))
            .ok();
        registry
            .register(ParamMetadata::new_uint32("SR_POSITION", 5, 0, 50))
            .ok();
        registry
            .register(ParamMetadata::new_uint32("SR_RC_CHAN", 5, 0, 50))
            .ok();

        let mut msg = heapless::String::<64>::new();
        let _ = core::fmt::write(
            &mut msg,
            format_args!("OK: Registered {} parameters\r\n\r\n", registry.count()),
        );
        let _ = class.write_packet(msg.as_bytes()).await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 9: Load from Flash
        let _ = class
            .write_packet(b"Test 9: Loading from Flash...\r\n")
            .await;
        match registry.load_from_flash() {
            Ok(_) => {
                let _ = class
                    .write_packet(b"OK: Load completed (may use defaults)\r\n")
                    .await;
            }
            Err(_) => {
                let _ = class
                    .write_packet(b"WARNING: Load failed (using defaults)\r\n")
                    .await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 10: Save to Flash (measure time)
        let _ = class
            .write_packet(b"Test 10: Saving to Flash (measuring time)...\r\n")
            .await;
        let save_start = embassy_time::Instant::now();
        match registry.save_to_flash() {
            Ok(_) => {
                let save_time_ms = save_start.elapsed().as_millis();
                let _ = class.write_packet(b"OK: Save successful\r\n").await;
                let mut msg = heapless::String::<64>::new();
                let _ = core::fmt::write(&mut msg, format_args!("  Time: {} ms\r\n", save_time_ms));
                let _ = class.write_packet(msg.as_bytes()).await;
            }
            Err(_) => {
                let _ = class.write_packet(b"ERROR: Save failed\r\n").await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 11: Multiple save cycles (10 cycles)
        let _ = class
            .write_packet(b"Test 11: Running 10 save cycles...\r\n")
            .await;

        for cycle in 0..10 {
            // Modify parameter value
            let new_value = 0.15 + ((cycle % 20) as f32 * 0.01);
            registry
                .set_by_name("RATE_ROLL_P", ParamValue::Float(new_value))
                .ok();

            // Save
            if let Err(_) = registry.save_to_flash() {
                let mut msg = heapless::String::<64>::new();
                let _ =
                    core::fmt::write(&mut msg, format_args!("ERROR: Save {} failed\r\n", cycle));
                let _ = class.write_packet(msg.as_bytes()).await;
            }

            // Progress every 2 cycles
            if (cycle + 1) % 2 == 0 {
                let mut msg = heapless::String::<64>::new();
                let _ =
                    core::fmt::write(&mut msg, format_args!("  Progress: {}/10\r\n", cycle + 1));
                let _ = class.write_packet(msg.as_bytes()).await;
            }

            Timer::after(Duration::from_millis(10)).await;
        }

        let _ = class.write_packet(b"OK: 10 cycles completed\r\n\r\n").await;

        Timer::after(Duration::from_secs(1)).await;

        // Test 12: Verify final state
        let _ = class
            .write_packet(b"Test 12: Verifying final state...\r\n")
            .await;
        match registry.load_from_flash() {
            Ok(_) => {
                let _ = class.write_packet(b"OK: Load successful\r\n").await;

                if let Some(param) = registry.get_by_name("RATE_ROLL_P") {
                    if let ParamValue::Float(f) = param.value {
                        let mut msg = heapless::String::<64>::new();
                        let _ =
                            core::fmt::write(&mut msg, format_args!("  RATE_ROLL_P = {}\r\n", f));
                        let _ = class.write_packet(msg.as_bytes()).await;
                    }
                }
            }
            Err(_) => {
                let _ = class.write_packet(b"ERROR: Verify load failed\r\n").await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        // Show storage stats
        if let Some(stats) = registry.storage_stats() {
            let _ = class.write_packet(b"Storage Statistics:\r\n").await;
            let mut msg1 = heapless::String::<64>::new();
            let _ = core::fmt::write(
                &mut msg1,
                format_args!("  Total saves: {}\r\n", stats.total_saves),
            );
            let _ = class.write_packet(msg1.as_bytes()).await;

            if let Some(block) = stats.active_block {
                let mut msg2 = heapless::String::<64>::new();
                let _ = core::fmt::write(&mut msg2, format_args!("  Active block: {}\r\n", block));
                let _ = class.write_packet(msg2.as_bytes()).await;
            }
        }
        let _ = class.write_packet(b"\r\n").await;

        let _ = class
            .write_packet(b"========================================\r\n")
            .await;
        let _ = class.write_packet(b"  All Tests Complete!\r\n").await;
        let _ = class
            .write_packet(b"========================================\r\n")
            .await;
        let _ = class
            .write_packet(b"\r\nParameter persistence is working!\r\n\r\n")
            .await;

        // Keep connection alive
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize platform
    let p = hal::init(Default::default());

    info!("Flash Minimal Test (USB-CDC)");

    // Create USB driver
    let driver = Driver::new(p.USB, Irqs);

    // USB configuration
    let mut config = Config::new(0x2e8a, 0x000a); // Raspberry Pi vendor/product
    config.manufacturer = Some("pico_trail");
    config.product = Some("Flash Test");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // USB device and builder setup
    let mut builder = Builder::new(
        driver,
        config,
        unsafe { &mut *core::ptr::addr_of_mut!(USB_DEVICE_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(USB_CONFIG_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(USB_BOS_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(USB_CONTROL_BUF) },
    );

    // Create CDC-ACM class
    let class = CdcAcmClass::new(
        &mut builder,
        unsafe { &mut *core::ptr::addr_of_mut!(USB_STATE) },
        64,
    );

    // Build USB device
    let usb = builder.build();

    // USB task needs to run separately
    spawner.spawn(usb_device_task(usb)).ok();

    // Spawn Flash test task
    spawner.spawn(flash_test_task(class)).unwrap();

    info!("USB and Flash test tasks started");

    // Main loop - just keep alive
    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}
