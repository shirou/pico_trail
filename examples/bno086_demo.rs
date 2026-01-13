//! BNO086 9-Axis IMU Demo (Interrupt Mode with GPIO)
//!
//! Hardware verification example for BNO086 sensor with on-chip sensor fusion.
//! Reads quaternion data via I2C with interrupt-driven acquisition and
//! hardware reset recovery.
//!
//! This demo uses **interrupt mode** with INT and RST pins for:
//! - True 100Hz data acquisition (no polling overhead)
//! - Automatic hardware reset recovery on errors
//! - Better stability and reliability
//!
//! # Hardware Setup
//!
//! Required connections:
//!
//! | BNO086 Pin | Pico 2 W Pin | GPIO | Description |
//! |------------|--------------|------|-------------|
//! | VCC        | 3V3          | -    | Power (3.3V) |
//! | GND        | GND          | -    | Ground |
//! | SDA        | Pin 6        | GP4  | I2C Data |
//! | SCL        | Pin 7        | GP5  | I2C Clock |
//! | INT        | Pin 29       | GP22 | Data ready interrupt (active low) |
//! | RST        | Pin 22       | GP17 | Hardware reset (active low) |
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
//! In interrupt mode with INT pin, the driver achieves true 100Hz data rate.
//! The INT signal triggers reads only when data is ready, eliminating polling
//! overhead and ensuring reliable timing.
//!
//! # Auto-Recovery
//!
//! The driver includes automatic hardware reset recovery:
//! - After 3 consecutive read errors, hardware reset is triggered
//! - Up to 3 consecutive resets are attempted before marking sensor as failed
//! - Recovery is transparent to the application

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_time::{Duration, Timer};
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

    pico_trail::log_info!("BNO086 Demo - GPIO Debug Mode");
    pico_trail::log_info!("=============================");
    pico_trail::log_info!("");

    // =========================================================================
    // GPIO Debug: Test INT (GPIO22) only - DO NOT touch RST
    // RST manipulation causes BNO086 to disappear from I2C bus
    // =========================================================================
    pico_trail::log_info!("=== GPIO Pin Test (INT only, RST untouched) ===");
    pico_trail::log_info!("INT = GPIO22");
    pico_trail::log_info!("");

    // Only configure INT as input - do NOT touch RST (GPIO17)
    let int = embassy_rp::gpio::Input::new(p.PIN_22, Pull::Up);

    // Read initial INT state
    let int_state = if int.is_low() { "LOW" } else { "HIGH" };
    pico_trail::log_info!("Initial INT state: {}", int_state);

    // Monitor INT for 2 seconds before I2C init
    pico_trail::log_info!("Monitoring INT for 2 seconds (before I2C)...");
    let mut last_int = int.is_high();
    let mut edge_count = 0;
    for i in 0..200 {
        Timer::after(Duration::from_millis(10)).await;
        let current_int = int.is_high();
        if last_int && !current_int {
            edge_count += 1;
            if edge_count <= 3 {
                pico_trail::log_info!("  Falling edge #{} at {}ms", edge_count, i * 10);
            }
        }
        last_int = current_int;
    }
    pico_trail::log_info!("  Edges before I2C: {}", edge_count);

    // Drop INT pin so we can check it again later
    drop(int);

    // =========================================================================
    // Step 1: I2C Bus Recovery
    // If I2C bus is still stuck, toggle SCL to release
    // =========================================================================
    pico_trail::log_info!("=== I2C Bus Recovery ===");

    // Configure SCL and SDA as outputs for bus recovery
    // We consume the pins here, then use unsafe steal() to get them back for I2C
    {
        let mut scl = Output::new(p.PIN_5, Level::High);
        let mut sda_out = Output::new(p.PIN_4, Level::High);

        // Wait a bit with both lines high
        Timer::after(Duration::from_millis(10)).await;

        // Release SDA to check its state
        drop(sda_out);
        let sda =
            embassy_rp::gpio::Input::new(unsafe { hal::peripherals::PIN_4::steal() }, Pull::Up);
        Timer::after(Duration::from_micros(100)).await;

        // Check if SDA is stuck low (slave holding it)
        if sda.is_low() {
            pico_trail::log_warn!("  SDA stuck low, attempting recovery...");

            // Toggle SCL up to 16 times with longer pulses to release stuck slave
            // BNO086 may need more clocks than typical I2C slaves
            for i in 0..16 {
                scl.set_low();
                Timer::after(Duration::from_micros(50)).await;
                scl.set_high();
                Timer::after(Duration::from_micros(50)).await;

                // Check if SDA released
                if sda.is_high() {
                    pico_trail::log_info!("  SDA released after {} clock pulses", i + 1);
                    break;
                }
            }

            // Generate STOP condition: SDA low->high while SCL high
            drop(sda);
            let mut sda_out2 = Output::new(unsafe { hal::peripherals::PIN_4::steal() }, Level::Low);
            Timer::after(Duration::from_micros(50)).await;
            scl.set_high();
            Timer::after(Duration::from_micros(50)).await;
            sda_out2.set_high(); // STOP: SDA rises while SCL high
            Timer::after(Duration::from_micros(100)).await;

            // Check SDA state again
            drop(sda_out2);
            let sda_check =
                embassy_rp::gpio::Input::new(unsafe { hal::peripherals::PIN_4::steal() }, Pull::Up);
            Timer::after(Duration::from_micros(100)).await;
            if sda_check.is_high() {
                pico_trail::log_info!("  Recovery complete, SDA is now HIGH");
            } else {
                pico_trail::log_warn!("  SDA still LOW after recovery - may need power cycle");
            }
        } else {
            pico_trail::log_info!("  SDA is high, bus OK");
        }
        // scl and sda dropped here, releasing the GPIO hardware
    }

    // Small delay after recovery
    Timer::after(Duration::from_millis(10)).await;

    // =========================================================================
    // Step 2: Initialize I2C (use unsafe steal to get pins back)
    // =========================================================================
    pico_trail::log_info!("Initializing I2C0 (GPIO4=SDA, GPIO5=SCL, 50kHz)...");

    // SAFETY: The GPIO pins were dropped above, so we can safely reclaim them
    let pin_4 = unsafe { hal::peripherals::PIN_4::steal() };
    let pin_5 = unsafe { hal::peripherals::PIN_5::steal() };

    let mut i2c0 = embassy_rp::i2c::I2c::new_async(
        p.I2C0,
        pin_5, // SCL
        pin_4, // SDA
        Irqs,
        {
            let mut config = embassy_rp::i2c::Config::default();
            config.frequency = 50_000; // 50kHz for stability after reset
            config
        },
    );

    // Brief delay for I2C bus to stabilize
    Timer::after(Duration::from_millis(100)).await;

    // =========================================================================
    // Step 3: Scan I2C bus for BNO086 (with retry)
    // =========================================================================
    let mut found_addr: Option<u8> = None;

    for attempt in 1..=3 {
        pico_trail::log_info!("Scanning I2C bus (attempt {}/3)...", attempt);

        // Only check BNO086 addresses (0x4A, 0x4B)
        for addr in [0x4B_u8, 0x4A] {
            let mut buf = [0u8; 1];
            match embassy_time::with_timeout(Duration::from_millis(100), i2c0.read(addr, &mut buf))
                .await
            {
                Ok(Ok(_)) => {
                    pico_trail::log_info!("  Found BNO086 at 0x{:02X}", addr);
                    found_addr = Some(addr);
                    break;
                }
                Ok(Err(_)) => {
                    pico_trail::log_debug!("  0x{:02X}: NACK", addr);
                }
                Err(_) => {
                    pico_trail::log_warn!("  0x{:02X}: timeout", addr);
                }
            }
        }

        if found_addr.is_some() {
            break;
        }

        if attempt < 3 {
            pico_trail::log_info!("  Not found, waiting 500ms before retry...");
            Timer::after(Duration::from_millis(500)).await;
        }
    }

    let bno_addr = match found_addr {
        Some(addr) => {
            pico_trail::log_info!("Using BNO086 at address 0x{:02X}", addr);
            addr
        }
        None => {
            pico_trail::log_error!("BNO086 not found on I2C bus!");
            pico_trail::log_error!("Check wiring: SDA=GPIO4, SCL=GPIO5, RST=GPIO17");
            pico_trail::log_error!("Halting.");
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    };

    // =========================================================================
    // Step 4: Create driver in POLLING mode
    // =========================================================================
    pico_trail::log_info!("Using POLLING mode (no INT/RST pins)...");

    // Create SHTP I2C transport
    let transport = pico_trail::communication::shtp::ShtpI2c::new(i2c0, bno_addr);

    // Create BNO086 driver in basic polling mode
    let config = pico_trail::devices::imu::bno086::Bno086Config::default();
    let mut driver = pico_trail::devices::imu::bno086::Bno086Driver::new(transport, config);

    pico_trail::log_info!("Reading raw SHTP packets (skip driver init)...");
    pico_trail::log_info!("BNO086 should output default reports without SET_FEATURE");
    pico_trail::log_info!("");

    // Release driver to get raw transport access
    let transport = driver.release();
    let mut shtp = transport;

    // Import SHTP types
    use pico_trail::communication::shtp::{ShtpPacket, ShtpTransport};

    // Read raw packets to see what sensor outputs by default
    pico_trail::log_info!("=== Raw packet reading (30 packets) ===");
    let mut quat_count = 0;
    for i in 0..30 {
        let mut packet = ShtpPacket::<280>::new();
        let read_result =
            embassy_time::with_timeout(Duration::from_millis(200), shtp.read_packet(&mut packet))
                .await;

        match read_result {
            Err(_) => {
                pico_trail::log_debug!("Pkt {}: timeout", i);
            }
            Ok(Err(e)) => {
                pico_trail::log_debug!("Pkt {}: {:?}", i, e);
            }
            Ok(Ok(())) => {
                let payload = packet.payload();
                let report_id = if !payload.is_empty() { payload[0] } else { 0 };

                // Only log interesting packets
                if packet.channel == 3 && payload.len() > 10 {
                    pico_trail::log_info!(
                        "Pkt {}: ch={} len={} report=0x{:02X}",
                        i,
                        packet.channel,
                        payload.len(),
                        report_id
                    );

                    // Check for quaternion
                    if report_id == 0x05 || report_id == 0x08 {
                        quat_count += 1;
                        pico_trail::log_info!("  -> Direct quaternion!");
                    }
                    if report_id == 0xFB && payload.len() > 15 {
                        // Check for embedded quaternion
                        for j in 5..payload.len().saturating_sub(10) {
                            if payload[j] == 0x08 || payload[j] == 0x05 {
                                quat_count += 1;
                                pico_trail::log_info!(
                                    "  -> Embedded 0x{:02X} at {}",
                                    payload[j],
                                    j
                                );
                                break;
                            }
                        }
                    }
                }
            }
        }
        Timer::after(Duration::from_millis(50)).await;
    }

    pico_trail::log_info!("");
    pico_trail::log_info!("Found {} quaternion reports", quat_count);

    if quat_count == 0 {
        pico_trail::log_error!("No quaternion data - sensor may need configuration");
        pico_trail::log_info!("Trying with driver init...");

        // Recreate driver and init
        let config = pico_trail::devices::imu::bno086::Bno086Config::default();
        let mut driver = pico_trail::devices::imu::bno086::Bno086Driver::new(shtp, config);

        match driver.init().await {
            Ok(()) => {
                pico_trail::log_info!("Driver initialized!");
                shtp = driver.release();
            }
            Err(e) => {
                pico_trail::log_error!("Init failed: {:?}", e);
                loop {
                    Timer::after(Duration::from_secs(60)).await;
                }
            }
        }

        // Read again after init
        pico_trail::log_info!("=== Reading after init (30 packets) ===");
        for i in 0..30 {
            let mut packet = ShtpPacket::<280>::new();
            let read_result = embassy_time::with_timeout(
                Duration::from_millis(200),
                shtp.read_packet(&mut packet),
            )
            .await;

            if let Ok(Ok(())) = read_result {
                let payload = packet.payload();
                let report_id = if !payload.is_empty() { payload[0] } else { 0 };
                if packet.channel == 3 && payload.len() > 10 {
                    pico_trail::log_info!(
                        "Pkt {}: ch={} len={} report=0x{:02X}",
                        i,
                        packet.channel,
                        payload.len(),
                        report_id
                    );
                }
            }
            Timer::after(Duration::from_millis(50)).await;
        }
    }

    pico_trail::log_info!("");
    pico_trail::log_info!("Demo complete. Halting.");
    loop {
        Timer::after(Duration::from_secs(60)).await;
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
