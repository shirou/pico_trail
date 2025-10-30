//! RP2350 platform implementation for Raspberry Pi Pico 2 W
//!
//! This module provides concrete implementations of the platform abstraction
//! traits for the RP2350 microcontroller using the `rp235x-hal` crate.
//!
//! # Feature Gate
//!
//! This module is only available when the `pico2_w` feature is enabled:
//!
//! ```toml
//! [dependencies]
//! pico_trail = { version = "0.1", features = ["pico2_w"] }
//! ```
//!
//! # Example
//!
//! ```no_run
//! use pico_trail::platform::rp2350::Rp2350Platform;
//! use pico_trail::platform::traits::Platform;
//!
//! let mut platform = Rp2350Platform::init();
//! let uart = platform.create_uart(/* config */);
//! ```

mod flash;
mod gpio;
mod i2c;
mod platform;
mod pwm;
mod spi;
mod timer;
mod uart;

pub use flash::Rp2350Flash;
pub use gpio::Rp2350Gpio;
pub use i2c::Rp2350I2c;
pub use platform::Rp2350Platform;
pub use pwm::Rp2350Pwm;
pub use spi::Rp2350Spi;
pub use timer::Rp2350Timer;
pub use uart::Rp2350Uart;
