//! Scheduler demonstration example with USB-CDC output
//!
//! This example demonstrates the Embassy-based task scheduler with multiple
//! tasks running at different rates, outputting statistics via USB serial.
//!
//! # Hardware
//!
//! This example is designed for Raspberry Pi Pico 2 W (RP2350).
//!
//! # Usage
//!
//! ```bash
//! cargo build --release --features pico2_w --example scheduler_demo_usb
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

use pico_trail::core::scheduler::monitor::monitor_task;
use pico_trail::core::scheduler::tasks::examples::{
    ahrs_task, control_task, imu_task, telemetry_task,
};

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

        let _ = class
            .write_packet(b"\r\n=== pico_trail Scheduler Demo ===\r\n")
            .await;
        let _ = class.write_packet(b"USB-CDC output version\r\n\r\n").await;
        let _ = class.write_packet(b"Tasks running:\r\n").await;
        let _ = class
            .write_packet(b"  - IMU task: 400Hz (2.5ms period)\r\n")
            .await;
        let _ = class
            .write_packet(b"  - AHRS task: 100Hz (10ms period)\r\n")
            .await;
        let _ = class
            .write_packet(b"  - Control task: 50Hz (20ms period)\r\n")
            .await;
        let _ = class
            .write_packet(b"  - Telemetry task: 10Hz (100ms period)\r\n")
            .await;
        let _ = class
            .write_packet(b"  - Monitor task: 1Hz (1s period)\r\n")
            .await;
        let _ = class
            .write_packet(b"\r\nMonitor statistics will appear here...\r\n\r\n")
            .await;

        loop {
            Timer::after(Duration::from_secs(1)).await;

            // Update CPU load (1 second window = 1,000,000 microseconds)
            let cpu_load = pico_trail::core::scheduler::update_cpu_load(1_000_000);

            // Get scheduler stats
            let sched_stats = pico_trail::core::scheduler::get_scheduler_stats();

            // Format and send statistics in small chunks (max 64 bytes per packet)
            let mut buf = heapless::String::<64>::new();

            // Send header
            buf.clear();
            let _ = core::fmt::write(
                &mut buf,
                format_args!(
                    "\r\n=== t={}s ===\r\n",
                    embassy_time::Instant::now().as_secs()
                ),
            );
            if class.write_packet(buf.as_bytes()).await.is_err() {
                info!("USB error");
                break;
            }

            // Send CPU load
            buf.clear();
            let _ = core::fmt::write(
                &mut buf,
                format_args!(
                    "CPU:{}% Miss:{}\r\n",
                    cpu_load, sched_stats.total_deadline_misses
                ),
            );
            if class.write_packet(buf.as_bytes()).await.is_err() {
                break;
            }

            // Send task statistics
            let task_count = pico_trail::core::scheduler::task_count();
            for i in 0..task_count {
                if let Some(metadata) = pico_trail::core::scheduler::get_task(i) {
                    let stats = pico_trail::core::scheduler::get_task_stats(i);
                    buf.clear();
                    let _ = core::fmt::write(
                        &mut buf,
                        format_args!(
                            "{}: {}us j={}us\r\n",
                            metadata.name, stats.avg_execution_us, stats.avg_jitter_us
                        ),
                    );
                    if class.write_packet(buf.as_bytes()).await.is_err() {
                        break;
                    }
                }
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize platform
    let p = hal::init(Default::default());

    info!("pico_trail Scheduler Demo (USB-CDC)");

    // Create USB driver
    let driver = Driver::new(p.USB, Irqs);

    // USB configuration
    let mut config = Config::new(0x2e8a, 0x000a); // Raspberry Pi vendor/product
    config.manufacturer = Some("pico_trail");
    config.product = Some("Scheduler Demo");
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

    // Spawn USB logger task
    spawner.spawn(usb_logger_task(class)).unwrap();

    // Wait a moment for USB to initialize
    Timer::after(Duration::from_millis(100)).await;

    info!("Starting scheduler tasks");

    // Spawn all scheduler tasks
    spawner.spawn(imu_task()).unwrap();
    spawner.spawn(ahrs_task()).unwrap();
    spawner.spawn(control_task()).unwrap();
    spawner.spawn(telemetry_task()).unwrap();
    spawner.spawn(monitor_task()).unwrap();

    info!("All tasks started successfully");

    // Main loop - just keep alive
    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}
