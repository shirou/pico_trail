//! u-blox GPS initialization functions
//!
//! Provides UBX protocol commands to configure u-blox GPS modules
//! (NEO-M8N and compatible) for NMEA output.
//!
//! # Example
//!
//! ```ignore
//! use pico_trail::devices::gps::{GpsDriver, init::ublox};
//!
//! let mut gps = GpsDriver::new(uart);
//! ublox::initialize(gps.uart_mut())?;
//! ```
//!
//! # References
//!
//! - [u-blox NEO-M8 Interface Description (UBX-13003221)](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
//! - [NeoGPS ubloxRate example](https://github.com/SlashDevin/NeoGPS/blob/master/examples/ubloxRate/ubloxRate.ino)

use crate::platform::{traits::UartInterface, Result};

/// Pre-built UBX commands following NeoGPS format (8-byte payload for CFG-MSG)
/// These use the extended format that sets rate for all ports explicitly.
///
/// Format: [class, id, len_lo, len_hi, msgClass, msgId, port0, port1, port2, port3, port4, port5]
/// Ports: I2C(0), UART1(1), UART2(2), USB(3), SPI(4), reserved(5)
/// Last byte (port5) is set to 1 for USB output in NeoGPS style
pub mod ubx_commands {
    use super::ubx_checksum;

    /// Disable GLL: 0x06,0x01,0x08,0x00, 0xF0,0x01, 0,0,0,0,0,1
    pub const DISABLE_GLL: [u8; 12] = [0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0, 0, 0, 0, 0, 1];
    /// Disable GSA: 0x06,0x01,0x08,0x00, 0xF0,0x02, 0,0,0,0,0,1
    pub const DISABLE_GSA: [u8; 12] = [0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0, 0, 0, 0, 0, 1];
    /// Disable GSV: 0x06,0x01,0x08,0x00, 0xF0,0x03, 0,0,0,0,0,1
    pub const DISABLE_GSV: [u8; 12] = [0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0, 0, 0, 0, 0, 1];
    /// Disable ZDA: 0x06,0x01,0x08,0x00, 0xF0,0x08, 0,0,0,0,0,1
    pub const DISABLE_ZDA: [u8; 12] = [0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0, 0, 0, 0, 0, 1];

    /// Enable GGA on UART1: rate=1
    pub const ENABLE_GGA: [u8; 12] = [0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0, 1, 0, 0, 0, 0];
    /// Enable RMC on UART1: rate=1
    pub const ENABLE_RMC: [u8; 12] = [0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0, 1, 0, 0, 0, 0];
    /// Enable VTG on UART1: rate=1
    pub const ENABLE_VTG: [u8; 12] = [0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0, 1, 0, 0, 0, 0];

    /// Send a UBX command (payload only, adds sync and checksum)
    pub fn send_ubx<U: super::UartInterface>(
        uart: &mut U,
        payload: &[u8; 12],
    ) -> super::Result<()> {
        // Write sync bytes
        uart.write(&[0xB5, 0x62])?;

        // Calculate checksum over payload
        let (ck_a, ck_b) = ubx_checksum(payload);

        // Write payload
        uart.write(payload)?;

        // Write checksum
        uart.write(&[ck_a, ck_b])?;

        Ok(())
    }
}

/// Initialize u-blox NEO-M8N GPS module with UBX commands
///
/// Sends UBX-CFG-MSG commands to configure NMEA output:
/// - Disables unnecessary messages (GSV, GSA, GLL, GST, ZDA) to reduce bandwidth
/// - Enables required messages: GGA, RMC, VTG at rate 1 on UART1
///
/// Uses 8-byte payload format (NeoGPS style) which explicitly sets rate for each port.
///
/// **Important**: Call this function after a startup delay (1000ms recommended)
/// to ensure the GPS module is ready to receive commands.
///
/// # Errors
///
/// Returns an error if UART write fails.
pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()> {
    use ubx_commands::*;

    // Disable all unnecessary NMEA messages first
    // This reduces UART bandwidth and prevents buffer overflow
    send_ubx(uart, &DISABLE_GLL)?;
    send_ubx(uart, &DISABLE_GSA)?;
    send_ubx(uart, &DISABLE_GSV)?;
    send_ubx(uart, &DISABLE_ZDA)?;

    // Enable only required messages on UART1
    send_ubx(uart, &ENABLE_GGA)?;
    send_ubx(uart, &ENABLE_RMC)?;
    send_ubx(uart, &ENABLE_VTG)?;

    Ok(())
}

/// Calculate UBX checksum (8-bit Fletcher algorithm)
pub(crate) fn ubx_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;

    for &byte in data {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }

    (ck_a, ck_b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ubx_checksum() {
        // Test with CFG-MSG payload for GGA
        // Class=0x06, ID=0x01, Len=0x0003, Payload=0xF0,0x00,0x01
        let data = [0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01];
        let (ck_a, ck_b) = ubx_checksum(&data);
        // Expected checksum can be verified against u-blox tools
        assert_eq!(ck_a, 0xFB);
        assert_eq!(ck_b, 0x11);
    }
}
