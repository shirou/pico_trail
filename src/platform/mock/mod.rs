//! Mock platform implementation for testing
//!
//! This module provides mock implementations of platform traits that can be used
//! for unit testing without requiring actual hardware.
//!
//! # Feature Gate
//!
//! This module is available in two contexts:
//! - During test builds (`#[cfg(test)]`)
//! - When the `mock` feature is enabled
//!
//! # Example
//!
//! ```
//! use pico_trail::platform::mock::{MockPlatform, MockUart};
//! use pico_trail::platform::traits::{Platform, UartInterface};
//!
//! let mut platform = MockPlatform::new();
//! let mut uart = platform.create_uart(0, Default::default()).unwrap();
//! uart.write(b"test")?;
//! ```

#![cfg(any(test, feature = "mock"))]

mod gpio;
mod i2c;
mod platform;
mod pwm;
mod spi;
mod timer;
mod uart;

pub use gpio::MockGpio;
pub use i2c::MockI2c;
pub use platform::MockPlatform;
pub use pwm::MockPwm;
pub use spi::MockSpi;
pub use timer::MockTimer;
pub use uart::MockUart;
