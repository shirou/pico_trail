//! BNO086 9-Axis IMU Driver with On-Chip Sensor Fusion
//!
//! I2C/SHTP driver for the Hillcrest/CEVA BNO086 IMU with integrated ARM Cortex-M0+
//! processor for sensor fusion. Unlike raw IMU sensors, the BNO086 provides
//! pre-computed quaternion output, eliminating the need for external EKF.
//!
//! ## Requirements
//!
//! - AN-srhcj: BNO086 IMU Integration Analysis
//! - T-x8mq2: BNO086 Driver Implementation
//!
//! ## Features
//!
//! - Quaternion output via Rotation Vector report
//! - 100Hz update rate (configurable)
//! - On-chip 9-axis sensor fusion
//! - SHTP protocol over I2C
//! - Interrupt-driven data acquisition (INT pin)
//! - Hardware reset recovery (RST pin)
//!
//! ## Hardware
//!
//! - 3-axis gyroscope
//! - 3-axis accelerometer
//! - 3-axis magnetometer
//! - ARM Cortex-M0+ for sensor fusion
//! - I2C @ 400kHz (Fast Mode)
//! - INT: Active low, falling edge = data ready
//! - RST: Active low = reset
//!
//! ## Driver Variants
//!
//! Two driver variants are provided:
//!
//! - `Bno086Driver<T>` - Basic polling mode driver
//! - `Bno086DriverWithGpio<T, INT, RST>` - Interrupt-driven with hardware reset
//!
//! ## Usage (Polling Mode)
//!
//! ```ignore
//! use pico_trail::devices::imu::bno086::{Bno086Driver, Bno086Config};
//! use pico_trail::communication::shtp::ShtpI2c;
//! use pico_trail::devices::traits::QuaternionSensor;
//!
//! let transport = ShtpI2c::new(i2c, 0x4A);
//! let mut driver = Bno086Driver::new(transport, Bno086Config::default());
//! driver.init().await?;
//!
//! let reading = driver.read_quaternion().await?;
//! ```
//!
//! ## Usage (Interrupt Mode with GPIO)
//!
//! ```ignore
//! use pico_trail::devices::imu::bno086::{
//!     Bno086DriverWithGpio, Bno086GpioConfig, EmbassyIntPin, EmbassyRstPin
//! };
//! use pico_trail::communication::shtp::ShtpI2c;
//! use pico_trail::devices::traits::QuaternionSensor;
//!
//! let transport = ShtpI2c::new(i2c, 0x4A);
//! let int_pin = EmbassyIntPin::new(gpio_int);
//! let rst_pin = EmbassyRstPin::new(gpio_rst);
//!
//! let mut driver = Bno086DriverWithGpio::new(
//!     transport, int_pin, rst_pin, Bno086GpioConfig::default()
//! );
//! driver.init().await?;
//!
//! loop {
//!     let reading = driver.read_quaternion().await?;
//!     // Process reading...
//! }
//! ```

mod driver;
mod driver_gpio;
mod gpio;
mod reports;

// Basic polling mode driver
pub use driver::{Bno086Config, Bno086Driver};

// GPIO-enabled interrupt mode driver
pub use driver_gpio::{Bno086DriverWithGpio, Bno086GpioConfig};

// GPIO abstractions
#[cfg(feature = "pico2_w")]
pub use gpio::{EmbassyIntPin, EmbassyRstPin};
pub use gpio::{IntPin, NoIntPin, NoRstPin, RstPin};

// Report types
pub use reports::{
    build_product_id_request, build_set_feature_command, ControlCommand, ProductIdResponse,
    ReportId, RotationVectorReport,
};
