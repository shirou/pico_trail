//! Platform abstraction layer
//!
//! This module provides hardware abstraction for different microcontroller platforms.
//! All platform-specific code must be isolated to this module per NFR-nmmu0.

pub mod error;
pub mod time;
pub mod traits;

// Platform implementations
pub mod rp2350;

#[cfg(any(test, feature = "mock"))]
pub mod mock;

// Re-export commonly used types
pub use error::{PlatformError, Result};
pub use time::EmbassyTime;
pub use traits::{
    AsyncUartInterface, FlashInterface, GpioInterface, I2cInterface, Platform, PwmInterface,
    SpiInterface, TimerInterface, UartInterface,
};
