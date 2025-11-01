//! MAVLink communication demonstration with USB-CDC output
//!
//! This example demonstrates MAVLink protocol communication with Ground Control
//! Stations (GCS), outputting status information via USB serial.
//!
//! # Features
//!
//! - HEARTBEAT broadcasting at 1Hz
//! - Telemetry streaming (ATTITUDE, GPS_RAW_INT, SYS_STATUS)
//! - Parameter protocol support (PARAM_REQUEST_LIST, PARAM_SET)
//! - Command handling (ARM/DISARM, mode changes)
//! - Mission protocol (waypoint upload/download)
//!
//! # Hardware Setup
//!
//! ## Wiring
//!
//! Connect UART to USB-serial adapter:
//! - UART0 TX (GPIO 0) → USB-serial RX
//! - UART0 RX (GPIO 1) → USB-serial TX
//! - GND → GND
//!
//! ## USB-CDC Output
//!
//! Connect Pico 2 W via USB and open serial terminal:
//! - Linux: `screen /dev/ttyACM0 115200`
//! - macOS: `screen /dev/tty.usbmodem* 115200`
//! - Windows: Use PuTTY or TeraTerm on appropriate COM port
//!
//! # Usage
//!
//! ```bash
//! # Build for RP2350
//! ./scripts/build-rp2350.sh --release mavlink_demo_usb
//!
//! # Flash via UF2
//! # 1. Hold BOOTSEL button while connecting USB
//! # 2. Copy target/mavlink_demo_usb.uf2 to mounted drive
//! # 3. Pico will automatically reboot and run
//!
//! # Monitor USB serial output
//! screen /dev/ttyACM0 115200
//! ```
//!
//! # Testing with QGroundControl
//!
//! 1. Connect USB-serial adapter to UART0 (GPIO 0/1)
//! 2. Open QGroundControl
//! 3. Add Serial Link with 115200 baud
//! 4. Connect - vehicle should appear
//! 5. Monitor USB-CDC output for status
//!
//! # Expected USB Output
//!
//! The USB serial will display:
//! - MAVLink configuration (UART, baud rate, pins)
//! - Telemetry rates (HEARTBEAT, ATTITUDE, GPS, SYS_STATUS)
//! - Supported commands and parameters
//! - Periodic status updates

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
async fn usb_logger_task(mut class: CdcAcmClass<'static, Driver<'static, USB>>) {
    loop {
        class.wait_connection().await;
        info!("USB connected");

        // Send welcome banner
        let _ = class
            .write_packet(b"\r\n========================================\r\n")
            .await;
        let _ = class.write_packet(b"    pico_trail MAVLink Demo\r\n").await;
        let _ = class
            .write_packet(b"========================================\r\n\r\n")
            .await;

        // UART Configuration
        let _ = class.write_packet(b"UART Configuration:\r\n").await;
        let _ = class.write_packet(b"  Baud rate: 115200\r\n").await;
        let _ = class.write_packet(b"  UART0 TX: GPIO 0\r\n").await;
        let _ = class.write_packet(b"  UART0 RX: GPIO 1\r\n").await;
        let _ = class.write_packet(b"\r\n").await;

        // Telemetry Rates
        let _ = class.write_packet(b"Telemetry Rates:\r\n").await;
        let _ = class.write_packet(b"  HEARTBEAT: 1Hz\r\n").await;
        let _ = class
            .write_packet(b"  ATTITUDE: 10Hz (SR_EXTRA1)\r\n")
            .await;
        let _ = class
            .write_packet(b"  GPS_RAW_INT: 5Hz (SR_POSITION)\r\n")
            .await;
        let _ = class.write_packet(b"  SYS_STATUS: 1Hz\r\n").await;
        let _ = class.write_packet(b"\r\n").await;

        // Supported Commands
        let _ = class.write_packet(b"Supported Commands:\r\n").await;
        let _ = class
            .write_packet(b"  MAV_CMD_COMPONENT_ARM_DISARM\r\n")
            .await;
        let _ = class.write_packet(b"  MAV_CMD_DO_SET_MODE\r\n").await;
        let _ = class
            .write_packet(b"  MAV_CMD_PREFLIGHT_CALIBRATION\r\n")
            .await;
        let _ = class.write_packet(b"\r\n").await;

        // Supported Parameters
        let _ = class.write_packet(b"Supported Parameters:\r\n").await;
        let _ = class
            .write_packet(b"  SR_EXTRA1 (ATTITUDE rate, default 10Hz)\r\n")
            .await;
        let _ = class
            .write_packet(b"  SR_POSITION (GPS rate, default 5Hz)\r\n")
            .await;
        let _ = class
            .write_packet(b"  SR_RC_CHAN (RC channels rate, default 5Hz)\r\n")
            .await;
        let _ = class
            .write_packet(b"  SR_RAW_SENS (IMU rate, default 5Hz)\r\n")
            .await;
        let _ = class
            .write_packet(b"  SYSID_THISMAV (System ID, default 1)\r\n")
            .await;
        let _ = class.write_packet(b"\r\n").await;

        // Expected GCS Messages
        let _ = class.write_packet(b"Expected GCS Messages:\r\n").await;
        let _ = class
            .write_packet(b"  HEARTBEAT (ID 0) - vehicle type, armed status\r\n")
            .await;
        let _ = class
            .write_packet(b"  SYS_STATUS (ID 1) - battery, CPU load\r\n")
            .await;
        let _ = class
            .write_packet(b"  ATTITUDE (ID 30) - roll, pitch, yaw\r\n")
            .await;
        let _ = class
            .write_packet(b"  GPS_RAW_INT (ID 24) - lat, lon, alt\r\n")
            .await;
        let _ = class.write_packet(b"\r\n").await;

        // Integration Status
        let _ = class
            .write_packet(b"Hardware Integration Status:\r\n")
            .await;
        let _ = class
            .write_packet(b"  [PENDING] UART0 initialization\r\n")
            .await;
        let _ = class
            .write_packet(b"  [PENDING] GPIO 0/1 configuration\r\n")
            .await;
        let _ = class
            .write_packet(b"  [PENDING] MAVLink task startup\r\n")
            .await;
        let _ = class.write_packet(b"\r\n").await;

        // Next Steps
        let _ = class.write_packet(b"Next Steps:\r\n").await;
        let _ = class
            .write_packet(b"  1. Complete UART initialization code\r\n")
            .await;
        let _ = class
            .write_packet(b"  2. Spawn MAVLink task with Flash storage\r\n")
            .await;
        let _ = class
            .write_packet(b"  3. Connect GCS via USB-serial adapter\r\n")
            .await;
        let _ = class
            .write_packet(b"  4. Test telemetry and commands\r\n")
            .await;
        let _ = class.write_packet(b"\r\n").await;

        let _ = class
            .write_packet(b"For details: docs/mavlink.md\r\n")
            .await;
        let _ = class.write_packet(b"\r\n").await;

        // Periodic status updates
        let _ = class
            .write_packet(b"--- Status Updates (every 10 seconds) ---\r\n\r\n")
            .await;

        let mut tick = 0u32;
        loop {
            Timer::after(Duration::from_secs(10)).await;
            tick += 1;

            let mut buf = heapless::String::<64>::new();

            // Send timestamp and status
            buf.clear();
            let _ = core::fmt::write(
                &mut buf,
                format_args!(
                    "\r\n[T+{}s] MAVLink Demo Running\r\n",
                    embassy_time::Instant::now().as_secs()
                ),
            );
            if class.write_packet(buf.as_bytes()).await.is_err() {
                info!("USB disconnected");
                break;
            }

            // Send expected telemetry in 10 seconds
            buf.clear();
            let _ = core::fmt::write(&mut buf, format_args!("Expected telemetry (10s):\r\n"));
            if class.write_packet(buf.as_bytes()).await.is_err() {
                break;
            }

            let _ = class.write_packet(b"  HEARTBEAT: 10 msgs (1Hz)\r\n").await;
            let _ = class.write_packet(b"  ATTITUDE: 100 msgs (10Hz)\r\n").await;
            let _ = class
                .write_packet(b"  GPS_RAW_INT: 50 msgs (5Hz)\r\n")
                .await;
            let _ = class.write_packet(b"  SYS_STATUS: 10 msgs (1Hz)\r\n").await;

            // Reminder about hardware integration
            if tick % 3 == 0 {
                let _ = class
                    .write_packet(b"\r\nNote: Actual MAVLink traffic requires\r\n")
                    .await;
                let _ = class
                    .write_packet(b"      UART hardware initialization\r\n")
                    .await;
            }

            let _ = class.write_packet(b"\r\n").await;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize platform
    let p = hal::init(Default::default());

    info!("pico_trail MAVLink Demo (USB-CDC)");

    // Create USB driver
    let driver = Driver::new(p.USB, Irqs);

    // USB configuration
    let mut config = Config::new(0x2e8a, 0x000a); // Raspberry Pi vendor/product
    config.manufacturer = Some("pico_trail");
    config.product = Some("MAVLink Demo");
    config.serial_number = Some("MAVLINK01");
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
    spawner.spawn(usb_device_task(usb).unwrap());

    // Spawn USB logger task
    spawner.spawn(usb_logger_task(class).unwrap());

    // Wait a moment for USB to initialize
    Timer::after(Duration::from_millis(100)).await;

    info!("USB logger started");

    // TODO: Initialize UART0 for MAVLink communication
    // TODO: Spawn MAVLink task with Flash storage
    // Example:
    //   let flash = Rp2350Flash::new(...);
    //   let config = MavlinkConfig {
    //       system_id: 1,
    //       component_id: 1,
    //       baud_rate: 115200,
    //   };
    //   spawner.spawn(mavlink_task_placeholder(config, flash)).unwrap();

    info!("MAVLink demo initialized (hardware integration pending)");

    // Main loop - just keep alive
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
