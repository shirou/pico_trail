//! BNO086 9-Axis IMU Demo (Polling Mode)
//!
//! Hardware verification example for BNO086 sensor with on-chip sensor fusion.
//! Reads quaternion data via I2C and outputs Euler angles.
//!
//! This demo uses **polling mode** (no INT/RST pins required).
//! Only VCC, GND, SDA, and SCL need to be connected.
//!
//! # Hardware Setup
//!
//! Minimum connections (polling mode):
//!
//! | BNO086 Pin | Pico 2 W Pin | GPIO | Description |
//! |------------|--------------|------|-------------|
//! | VCC        | 3V3          | -    | Power (3.3V) |
//! | GND        | GND          | -    | Ground |
//! | SDA        | Pin 6        | GP4  | I2C Data |
//! | SCL        | Pin 7        | GP5  | I2C Clock |
//!
//! Optional connections (for interrupt mode - not used in this demo):
//! - INT (GP6): Data ready interrupt
//! - RST (GP7): Hardware reset
//!
//! # Usage
//!
//! ```bash
//! # Build for RP2350 (defmt logging via probe-rs)
//! ./scripts/build-rp2350.sh bno086_demo
//!
//! # Flash and run with probe-rs (shows defmt output)
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/debug/examples/bno086_demo
//!
//! # Build with USB Serial (no probe-rs needed, appears as /dev/ttyACM0)
//! EXTRA_FEATURES="usb_serial" ./scripts/build-rp2350.sh bno086_demo
//!
//! # Flash via BOOTSEL: Hold BOOTSEL, connect USB, copy target/bno086_demo.uf2
//! # View output: screen /dev/ttyACM0 115200
//! ```
//!
//! # Expected Output
//!
//! ```text
//! BNO086 Demo
//! ===========
//! Initializing I2C0 (GPIO4=SDA, GPIO5=SCL)...
//! Initializing BNO086 driver...
//! BNO086: Product ID - SW x.y.z
//! BNO086: Initialized successfully
//!
//! Reading quaternion data at 100Hz...
//! Euler: Roll=0° Pitch=-1° Yaw=45°
//! Stats: 1000 samples, 0 errors, ~40Hz, healthy=true
//! ```
//!
//! # Note on Sample Rate
//!
//! In polling mode (no INT pin), the effective rate is ~40Hz due to I2C polling
//! overhead. For 100Hz operation, use interrupt mode with INT pin connected.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c as I2cTrait;

// USB serial support (when built with usb_serial feature)
#[cfg(feature = "usb_serial")]
use core::ptr::addr_of_mut;
#[cfg(feature = "usb_serial")]
use embassy_rp::usb::{Driver, InterruptHandler};
#[cfg(feature = "usb_serial")]
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
#[cfg(feature = "usb_serial")]
use embassy_usb::{Builder, Config};

// Panic handler for USB serial builds
#[cfg(feature = "usb_serial")]
use panic_halt as _;

// Panic handler and defmt-rtt for non-USB builds
#[cfg(not(feature = "usb_serial"))]
use {defmt_rtt as _, panic_probe as _};

// Global allocator (required by pico_trail crate)
#[cfg(feature = "pico2_w")]
use embedded_alloc::LlffHeap as Heap;

#[cfg(feature = "pico2_w")]
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[cfg(feature = "pico2_w")]
const HEAP_SIZE: usize = 4 * 1024; // 4 KB (minimal for this demo)

// IRQ bindings: USB + I2C0 when usb_serial is enabled, I2C0 only otherwise
#[cfg(feature = "usb_serial")]
hal::bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<hal::peripherals::USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<hal::peripherals::I2C0>;
});

#[cfg(not(feature = "usb_serial"))]
hal::bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<hal::peripherals::I2C0>;
});

#[embassy_executor::main]
async fn main(#[allow(unused)] spawner: Spawner) {
    // Initialize heap for allocations
    #[cfg(feature = "pico2_w")]
    {
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
        unsafe {
            HEAP.init(
                core::ptr::addr_of_mut!(HEAP_MEM) as *mut u8 as usize,
                HEAP_SIZE,
            )
        }
    }

    // Initialize peripherals
    let p = hal::init(Default::default());

    // Initialize USB Serial (if feature enabled)
    #[cfg(feature = "usb_serial")]
    {
        let driver = Driver::new(p.USB, Irqs);

        let mut usb_config = Config::new(0x2e8a, 0x000a);
        usb_config.manufacturer = Some("Raspberry Pi");
        usb_config.product = Some("BNO086 Demo");
        usb_config.serial_number = Some("12345678");
        usb_config.max_power = 100;
        usb_config.max_packet_size_0 = 64;

        static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
        static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
        static mut CONTROL_BUF: [u8; 64] = [0; 64];
        static mut MSOS_DESCRIPTOR: [u8; 256] = [0; 256];
        static mut STATE: State = State::new();

        let mut builder = Builder::new(
            driver,
            usb_config,
            unsafe { &mut *addr_of_mut!(CONFIG_DESCRIPTOR) },
            unsafe { &mut *addr_of_mut!(BOS_DESCRIPTOR) },
            unsafe { &mut *addr_of_mut!(MSOS_DESCRIPTOR) },
            unsafe { &mut *addr_of_mut!(CONTROL_BUF) },
        );

        let cdc_class = CdcAcmClass::new(&mut builder, unsafe { &mut *addr_of_mut!(STATE) }, 64);
        let usb = builder.build();

        // Spawn USB logger task
        spawner.spawn(pico_trail::core::logging::usb_logger_task(cdc_class).unwrap());

        // Spawn USB device task
        spawner.spawn(usb_task(usb).unwrap());

        // Wait for USB to enumerate before proceeding
        Timer::after(Duration::from_secs(1)).await;
        pico_trail::log_info!("USB initialized, waiting for terminal...");
    }

    // Wait 5 seconds at startup to allow serial terminal connection
    pico_trail::log_info!("Starting in 5 seconds...");
    Timer::after(Duration::from_secs(5)).await;

    pico_trail::log_info!("BNO086 Demo");
    pico_trail::log_info!("===========");
    pico_trail::log_info!("");

    // Initialize I2C0 for BNO086 (GPIO4 = SDA, GPIO5 = SCL)
    pico_trail::log_info!("Initializing I2C0 (GPIO4=SDA, GPIO5=SCL)...");
    let mut i2c0 = embassy_rp::i2c::I2c::new_async(
        p.I2C0,
        p.PIN_5, // SCL
        p.PIN_4, // SDA
        Irqs,
        {
            let mut config = embassy_rp::i2c::Config::default();
            config.frequency = 100_000; // 100kHz Standard Mode (more stable)
            config
        },
    );

    // Wait for BNO086 to power up (needs time after power-on)
    pico_trail::log_info!("Waiting for sensor power-up...");
    Timer::after(Duration::from_millis(500)).await;

    // Scan I2C bus for devices
    pico_trail::log_info!("Scanning I2C bus for devices...");
    let mut found_addr: Option<u8> = None;
    for addr in 0x08..0x78 {
        let mut buf = [0u8; 1];
        if i2c0.read(addr, &mut buf).await.is_ok() {
            pico_trail::log_info!("  Found device at 0x{:02X}", addr);
            if addr == 0x4A || addr == 0x4B {
                found_addr = Some(addr);
            }
        }
    }

    let bno_addr = match found_addr {
        Some(addr) => {
            pico_trail::log_info!("Using BNO086 at address 0x{:02X}", addr);
            addr
        }
        None => {
            pico_trail::log_warn!("No BNO086 found, trying default 0x4A");
            0x4A
        }
    };

    // Create SHTP I2C transport
    let transport = pico_trail::communication::shtp::ShtpI2c::new(i2c0, bno_addr);

    // Create BNO086 driver in polling mode (no INT/RST pins required)
    // Note: INT/RST pins improve efficiency and recovery but are optional
    let config = pico_trail::devices::imu::bno086::Bno086Config::default();
    let mut driver = pico_trail::devices::imu::bno086::Bno086Driver::new(transport, config);

    pico_trail::log_info!("Initializing BNO086 driver...");

    // Initialize the sensor
    pico_trail::log_info!("Calling driver.init()...");
    match driver.init().await {
        Ok(()) => {
            pico_trail::log_info!("BNO086 initialized successfully!");
        }
        Err(e) => {
            pico_trail::log_error!("BNO086 initialization failed: {:?}", e);
            pico_trail::log_error!("Possible causes:");
            pico_trail::log_error!("  1. Check I2C wiring (SDA=GP4, SCL=GP5)");
            pico_trail::log_error!("  2. Try address 0x4B if SA0 pin is high");
            pico_trail::log_error!("  3. Ensure BNO086 has 3.3V power");
            pico_trail::log_error!("Retrying in 5 seconds...");
            Timer::after(Duration::from_secs(5)).await;

            // Retry once
            pico_trail::log_info!("Retrying initialization...");
            match driver.init().await {
                Ok(()) => {
                    pico_trail::log_info!("BNO086 initialized on retry!");
                }
                Err(e2) => {
                    pico_trail::log_error!("Retry failed: {:?}", e2);
                    loop {
                        Timer::after(Duration::from_secs(60)).await;
                    }
                }
            }
        }
    }

    // Print product ID if available
    if let Some(product_id) = driver.product_id() {
        pico_trail::log_info!(
            "Product ID: SW version {}.{}.{}",
            product_id.sw_version_major,
            product_id.sw_version_minor,
            product_id.sw_version_patch
        );
    }

    pico_trail::log_info!("");
    pico_trail::log_info!("Reading quaternion data at 100Hz...");
    pico_trail::log_info!("Rotate sensor to see Euler angle changes");
    pico_trail::log_info!("");

    // Use QuaternionSensor trait
    use pico_trail::devices::traits::QuaternionSensor;

    // Statistics for rate measurement
    let mut sample_count: u32 = 0;
    let mut error_count: u32 = 0;
    let start_time = Instant::now();
    let mut last_stats_time = start_time;

    loop {
        // Read quaternion from sensor
        match driver.read_quaternion().await {
            Ok(reading) => {
                sample_count += 1;

                // Convert quaternion to Euler angles (roll, pitch, yaw)
                let (roll, pitch, yaw) = quaternion_to_euler(
                    reading.quaternion.w,
                    reading.quaternion.i,
                    reading.quaternion.j,
                    reading.quaternion.k,
                );

                // Log every 10th sample (~10Hz output)
                if sample_count % 10 == 0 {
                    let roll_i = roll as i32;
                    let pitch_i = pitch as i32;
                    let yaw_i = yaw as i32;
                    pico_trail::log_info!(
                        "Euler: Roll={}° Pitch={}° Yaw={}°",
                        roll_i,
                        pitch_i,
                        yaw_i
                    );
                }

                // Print statistics every 5 seconds
                let now = Instant::now();
                if now.duration_since(last_stats_time) >= Duration::from_secs(5) {
                    let elapsed_secs = now.duration_since(start_time).as_secs();
                    let rate = if elapsed_secs > 0 {
                        sample_count / elapsed_secs as u32
                    } else {
                        0
                    };

                    pico_trail::log_info!(
                        "Stats: {} samples, {} errors, ~{}Hz, healthy={}",
                        sample_count,
                        error_count,
                        rate,
                        driver.is_healthy()
                    );
                    last_stats_time = now;
                }
            }
            Err(e) => {
                error_count += 1;
                if error_count % 10 == 1 {
                    // Only log every 10th error to avoid spam
                    pico_trail::log_warn!("Read error: {:?} (count={})", e, error_count);
                }

                // Longer delay after errors to let I2C bus recover
                Timer::after(Duration::from_millis(50)).await;
            }
        }

        // Delay between reads - don't poll too aggressively
        // At 100Hz, data arrives every 10ms, so 5ms delay is reasonable
        Timer::after(Duration::from_millis(5)).await;
    }
}

/// Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) in degrees
///
/// Uses aerospace convention:
/// - Roll: rotation about X-axis (positive = right wing down)
/// - Pitch: rotation about Y-axis (positive = nose up)
/// - Yaw: rotation about Z-axis (positive = clockwise when viewed from above)
fn quaternion_to_euler(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    use libm::{asinf, atan2f};

    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        // Use 90 degrees if out of range
        core::f32::consts::FRAC_PI_2.copysign(sinp)
    } else {
        asinf(sinp)
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = atan2f(siny_cosp, cosy_cosp);

    // Convert to degrees
    let to_deg = 180.0 / core::f32::consts::PI;
    (roll * to_deg, pitch * to_deg, yaw * to_deg)
}

/// USB device task
///
/// Runs the USB device event loop for USB Serial functionality.
#[cfg(feature = "usb_serial")]
#[embassy_executor::task]
async fn usb_task(
    mut usb: embassy_usb::UsbDevice<'static, Driver<'static, hal::peripherals::USB>>,
) {
    usb.run().await;
}
