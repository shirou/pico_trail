//! RP2350 SPI implementation
//!
//! This module provides SPI support for RP2350 using the `rp235x-hal` crate.

use crate::platform::{
    Result,
    error::{PlatformError, SpiError},
    traits::{SpiConfig, SpiInterface},
};
use rp235x_hal::spi::Spi;

/// RP2350 SPI implementation
///
/// Wraps the `rp235x-hal` SPI peripheral to implement the `SpiInterface` trait.
///
/// # Note
///
/// Chip select (CS) management is separate and typically done via GPIO.
pub struct Rp2350Spi<D, P>
where
    D: rp235x_hal::spi::SpiDevice,
    P: rp235x_hal::spi::ValidSpiPinout<D>,
{
    spi: Spi<rp235x_hal::spi::Enabled, D, P, 8>,
}

impl<D, P> Rp2350Spi<D, P>
where
    D: rp235x_hal::spi::SpiDevice,
    P: rp235x_hal::spi::ValidSpiPinout<D>,
{
    /// Create a new RP2350 SPI instance
    ///
    /// # Arguments
    ///
    /// * `spi` - The HAL SPI peripheral
    /// * `config` - SPI configuration
    pub fn new(spi: Spi<rp235x_hal::spi::Enabled, D, P, 8>, _config: SpiConfig) -> Self {
        // Note: Configuration is typically done during peripheral initialization
        // The mode and frequency are set via the HAL's SPI constructor
        Self { spi }
    }
}

impl<D, P> SpiInterface for Rp2350Spi<D, P>
where
    D: rp235x_hal::spi::SpiDevice,
    P: rp235x_hal::spi::ValidSpiPinout<D>,
{
    fn transfer(&mut self, write_buffer: &[u8], read_buffer: &mut [u8]) -> Result<()> {
        use embedded_hal::blocking::spi::Transfer;

        if write_buffer.len() != read_buffer.len() {
            return Err(PlatformError::Spi(SpiError::TransferFailed));
        }

        // Copy write_buffer to read_buffer first, as transfer is in-place
        read_buffer.copy_from_slice(write_buffer);

        self.spi
            .transfer(read_buffer)
            .map_err(|_| PlatformError::Spi(SpiError::TransferFailed))?;

        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        use embedded_hal::blocking::spi::Write;

        self.spi
            .write(data)
            .map_err(|_| PlatformError::Spi(SpiError::TransferFailed))
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<()> {
        use embedded_hal::blocking::spi::Transfer;

        // Fill buffer with dummy bytes (0x00)
        for byte in buffer.iter_mut() {
            *byte = 0x00;
        }

        self.spi
            .transfer(buffer)
            .map_err(|_| PlatformError::Spi(SpiError::TransferFailed))?;

        Ok(())
    }

    fn set_frequency(&mut self, _frequency: u32) -> Result<()> {
        // The RP2350 HAL doesn't provide a runtime frequency change method
        // Frequency is set during initialization
        // For now, we return success as the frequency is set during peripheral creation
        Ok(())
    }
}
