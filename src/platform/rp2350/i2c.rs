//! RP2350 I2C implementation
//!
//! This module provides async I2C support for RP2350 using Embassy's `embassy-rp` crate.
//!
//! # Example
//!
//! ```no_run
//! use embassy_rp::i2c::Config as EmbassyI2cConfig;
//! use pico_trail::platform::{traits::{I2cConfig, I2cInterface}, rp2350::Rp2350I2c};
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: embassy_executor::Spawner) {
//!     let p = embassy_rp::init(Default::default());
//!
//!     // Initialize I2C0 on GPIO 0 (SDA) and GPIO 1 (SCL) at 400 kHz
//!     let config = I2cConfig {
//!         frequency: 400_000,
//!         timeout_us: 1_000_000,
//!     };
//!
//!     let embassy_config = EmbassyI2cConfig::default();
//!     let i2c = embassy_rp::i2c::I2c::new_async(
//!         p.I2C0,
//!         p.PIN_0,  // SDA
//!         p.PIN_1,  // SCL
//!         embassy_rp::bind_interrupts!(struct Irqs { I2C0_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C0>; }),
//!         embassy_config,
//!     );
//!
//!     let mut i2c = Rp2350I2c::new(i2c, config);
//!
//!     // Read from device at address 0x42
//!     let mut buffer = [0u8; 10];
//!     i2c.read(0x42, &mut buffer).await.unwrap();
//! }
//! ```

use crate::platform::{
    error::{I2cError, PlatformError},
    traits::{I2cConfig, I2cInterface},
    Result,
};
use embassy_rp::i2c::{Async, I2c as EmbassyI2c, Instance};
use embedded_hal_async::i2c::I2c as AsyncI2cTrait;

/// RP2350 I2C implementation using Embassy async I2C
///
/// Wraps the `embassy-rp` I2C peripheral to implement the async `I2cInterface` trait.
///
/// # Type Parameters
///
/// * `T` - I2C peripheral instance (I2C0 or I2C1)
pub struct Rp2350I2c<'d, T: Instance> {
    i2c: EmbassyI2c<'d, T, Async>,
    _config: I2cConfig,
}

impl<'d, T: Instance> Rp2350I2c<'d, T> {
    /// Create a new RP2350 I2C instance
    ///
    /// # Arguments
    ///
    /// * `i2c` - The Embassy I2C peripheral (configured via `embassy_rp::i2c::I2c::new_async`)
    /// * `config` - I2C configuration (frequency, timeout)
    ///
    /// # Note
    ///
    /// The I2C peripheral should be initialized with `embassy_rp::i2c::I2c::new_async`
    /// before being passed to this constructor. The frequency setting in `config` is
    /// informational only; actual frequency must be set during Embassy I2C initialization.
    pub fn new(i2c: EmbassyI2c<'d, T, Async>, config: I2cConfig) -> Self {
        Self {
            i2c,
            _config: config,
        }
    }
}

impl<'d, T: Instance> I2cInterface for Rp2350I2c<'d, T> {
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<()> {
        self.i2c
            .write(addr, data)
            .await
            .map_err(|e| map_embassy_error(e))
    }

    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<()> {
        self.i2c
            .read(addr, buffer)
            .await
            .map_err(|e| map_embassy_error(e))
    }

    async fn write_read(
        &mut self,
        addr: u8,
        write_data: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<()> {
        self.i2c
            .write_read(addr, write_data, read_buffer)
            .await
            .map_err(|e| map_embassy_error(e))
    }

    fn set_frequency(&mut self, _frequency: u32) -> Result<()> {
        // Embassy I2C doesn't provide a runtime frequency change method
        // Frequency is set during initialization via embassy_rp::i2c::Config
        // For now, we return success as the frequency is set during peripheral creation
        Ok(())
    }
}

/// Map Embassy I2C errors to platform I2C errors
#[allow(deprecated)]
fn map_embassy_error(error: embassy_rp::i2c::Error) -> PlatformError {
    use embassy_rp::i2c::Error;

    match error {
        Error::Abort(abort_reason) => {
            use embassy_rp::i2c::AbortReason;
            match abort_reason {
                AbortReason::NoAcknowledge => PlatformError::I2c(I2cError::Nack),
                AbortReason::ArbitrationLoss => PlatformError::I2c(I2cError::ArbitrationLost),
                _ => PlatformError::I2c(I2cError::BusError),
            }
        }
        Error::InvalidReadBufferLength | Error::InvalidWriteBufferLength => {
            PlatformError::I2c(I2cError::BusError)
        }
        Error::AddressOutOfRange(_) | Error::AddressReserved(_) => {
            PlatformError::I2c(I2cError::InvalidAddress)
        }
    }
}
