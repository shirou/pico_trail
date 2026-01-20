//! RP2350 UART implementation
//!
//! This module provides UART support for RP2350 using the `rp235x-hal` crate.

use crate::platform::{
    error::{PlatformError, UartError},
    traits::{UartConfig, UartInterface},
    Result,
};
use rp235x_hal::uart::UartPeripheral;

/// RP2350 UART implementation
///
/// Wraps the `rp235x-hal` UART peripheral to implement the `UartInterface` trait.
///
/// # Note
///
/// In rp235x-hal 0.3.0, UART configuration is done during the `enable()` call,
/// not through separate setter methods. This wrapper assumes the UART peripheral
/// has already been enabled with the desired configuration.
///
/// # Example
///
/// ```no_run
/// use pico_trail::platform::rp2350::Rp2350Uart;
/// use pico_trail::platform::traits::{UartInterface, UartConfig};
///
/// // UART should already be enabled with configuration
/// let mut uart = Rp2350Uart::new(enabled_uart, config);
/// uart.write(b"Hello, World!")?;
/// ```
pub struct Rp2350Uart<D, P>
where
    D: rp235x_hal::uart::UartDevice,
    P: rp235x_hal::uart::ValidUartPinout<D>,
{
    uart: UartPeripheral<rp235x_hal::uart::Enabled, D, P>,
}

impl<D, P> Rp2350Uart<D, P>
where
    D: rp235x_hal::uart::UartDevice,
    P: rp235x_hal::uart::ValidUartPinout<D>,
{
    /// Create a new RP2350 UART instance
    ///
    /// # Arguments
    ///
    /// * `uart` - The HAL UART peripheral (already enabled)
    /// * `_config` - UART configuration (ignored, as UART is already configured)
    ///
    /// # Note
    ///
    /// The UART peripheral should already be enabled with the desired configuration
    /// using `UartPeripheral::enable()` before passing it to this constructor.
    pub fn new(uart: UartPeripheral<rp235x_hal::uart::Enabled, D, P>, _config: UartConfig) -> Self {
        // Configuration is done during enable(), not here
        Self { uart }
    }
}

impl<D, P> UartInterface for Rp2350Uart<D, P>
where
    D: rp235x_hal::uart::UartDevice,
    P: rp235x_hal::uart::ValidUartPinout<D>,
{
    fn write(&mut self, data: &[u8]) -> Result<usize> {
        // write_full_blocking() writes entire buffer
        self.uart.write_full_blocking(data);
        Ok(data.len())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        // read_raw() reads available bytes without blocking
        let read_count = self
            .uart
            .read_raw(buffer)
            .map_err(|_| PlatformError::Uart(UartError::ReadFailed))?;
        Ok(read_count)
    }

    fn set_baud_rate(&mut self, _baud: u32) -> Result<()> {
        // rp235x-hal 0.3.0 doesn't support runtime baud rate changes
        // Would need to disable and re-enable the UART with new config
        Err(PlatformError::Uart(UartError::InvalidBaudRate))
    }

    fn available(&self) -> bool {
        self.uart.uart_is_readable()
    }

    fn flush(&mut self) -> Result<()> {
        // Wait until UART is no longer busy (all data transmitted)
        while self.uart.uart_is_busy() {
            // Busy wait
        }
        Ok(())
    }
}
