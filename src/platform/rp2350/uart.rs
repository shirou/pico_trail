//! RP2350 UART implementation
//!
//! This module provides UART support for RP2350 using the `rp235x-hal` crate.

use crate::platform::{
    Result,
    error::{PlatformError, UartError},
    traits::{UartConfig, UartInterface, UartParity, UartStopBits},
};
use rp235x_hal::uart::{DataBits, Parity, StopBits, UartPeripheral};

/// RP2350 UART implementation
///
/// Wraps the `rp235x-hal` UART peripheral to implement the `UartInterface` trait.
///
/// # Example
///
/// ```no_run
/// use pico_trail::platform::rp2350::Rp2350Uart;
/// use pico_trail::platform::traits::{UartInterface, UartConfig};
///
/// let mut uart = Rp2350Uart::new(hal_uart, config);
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
    /// * `uart` - The HAL UART peripheral
    /// * `config` - UART configuration
    pub fn new(
        mut uart: UartPeripheral<rp235x_hal::uart::Enabled, D, P>,
        config: UartConfig,
    ) -> Self {
        // Configure UART parameters
        let data_bits = match config.data_bits {
            8 => DataBits::Eight,
            7 => DataBits::Seven,
            6 => DataBits::Six,
            5 => DataBits::Five,
            _ => DataBits::Eight, // Default to 8 bits
        };

        let parity = match config.parity {
            UartParity::None => Parity::None,
            UartParity::Even => Parity::Even,
            UartParity::Odd => Parity::Odd,
        };

        let stop_bits = match config.stop_bits {
            UartStopBits::One => StopBits::One,
            UartStopBits::Two => StopBits::Two,
        };

        uart.set_format(data_bits, stop_bits, parity);

        Self { uart }
    }
}

impl<D, P> UartInterface for Rp2350Uart<D, P>
where
    D: rp235x_hal::uart::UartDevice,
    P: rp235x_hal::uart::ValidUartPinout<D>,
{
    fn write(&mut self, data: &[u8]) -> Result<usize> {
        let mut written = 0;
        for &byte in data {
            // Write byte, blocking until TX FIFO has space
            self.uart
                .write_full_blocking(byte)
                .map_err(|_| PlatformError::Uart(UartError::WriteFailed))?;
            written += 1;
        }
        Ok(written)
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        let mut read_count = 0;
        for slot in buffer.iter_mut() {
            // Non-blocking read
            match self.uart.read() {
                Ok(byte) => {
                    *slot = byte;
                    read_count += 1;
                }
                Err(nb::Error::WouldBlock) => {
                    // No more data available
                    break;
                }
                Err(_) => {
                    return Err(PlatformError::Uart(UartError::ReadFailed));
                }
            }
        }
        Ok(read_count)
    }

    fn set_baud_rate(&mut self, baud: u32) -> Result<()> {
        // RP2350 system clock is typically 125 MHz
        const SYS_CLOCK_HZ: u32 = 125_000_000;
        self.uart
            .set_baudrate(baud, fugit::HertzU32::Hz(SYS_CLOCK_HZ))
            .map_err(|_| PlatformError::Uart(UartError::InvalidBaudRate))?;
        Ok(())
    }

    fn available(&self) -> bool {
        self.uart.uart_is_readable()
    }

    fn flush(&mut self) -> Result<()> {
        self.uart.transmit_flushed();
        Ok(())
    }
}
