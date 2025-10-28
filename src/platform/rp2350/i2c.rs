//! RP2350 I2C implementation
//!
//! This module provides I2C support for RP2350 using the `rp235x-hal` crate.

use crate::platform::{
    Result,
    error::{I2cError, PlatformError},
    traits::{I2cConfig, I2cInterface},
};
use rp235x_hal::i2c::I2C;

/// RP2350 I2C implementation
///
/// Wraps the `rp235x-hal` I2C peripheral to implement the `I2cInterface` trait.
pub struct Rp2350I2c<D, P>
where
    D: rp235x_hal::i2c::I2CDevice,
    P: rp235x_hal::i2c::ValidI2CPinout<D>,
{
    i2c: I2C<D, P>,
}

impl<D, P> Rp2350I2c<D, P>
where
    D: rp235x_hal::i2c::I2CDevice,
    P: rp235x_hal::i2c::ValidI2CPinout<D>,
{
    /// Create a new RP2350 I2C instance
    ///
    /// # Arguments
    ///
    /// * `i2c` - The HAL I2C peripheral
    /// * `config` - I2C configuration
    pub fn new(i2c: I2C<D, P>, _config: I2cConfig) -> Self {
        // Note: Configuration is typically done during peripheral initialization
        // The frequency is set via the HAL's I2C constructor
        Self { i2c }
    }
}

impl<D, P> I2cInterface for Rp2350I2c<D, P>
where
    D: rp235x_hal::i2c::I2CDevice,
    P: rp235x_hal::i2c::ValidI2CPinout<D>,
{
    fn write(&mut self, addr: u8, data: &[u8]) -> Result<()> {
        use embedded_hal::blocking::i2c::Write;

        self.i2c
            .write(addr, data)
            .map_err(|_| PlatformError::I2c(I2cError::BusError))
    }

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<()> {
        use embedded_hal::blocking::i2c::Read;

        self.i2c
            .read(addr, buffer)
            .map_err(|_| PlatformError::I2c(I2cError::BusError))
    }

    fn write_read(&mut self, addr: u8, write_data: &[u8], read_buffer: &mut [u8]) -> Result<()> {
        use embedded_hal::blocking::i2c::WriteRead;

        self.i2c
            .write_read(addr, write_data, read_buffer)
            .map_err(|_| PlatformError::I2c(I2cError::BusError))
    }

    fn set_frequency(&mut self, _frequency: u32) -> Result<()> {
        // The RP2350 HAL doesn't provide a runtime frequency change method
        // Frequency is set during initialization
        // For now, we return success as the frequency is set during peripheral creation
        Ok(())
    }
}
