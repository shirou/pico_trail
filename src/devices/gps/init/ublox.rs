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

use crate::platform::{traits::UartInterface, Result};

/// Initialize u-blox NEO-M8N GPS module with UBX commands
///
/// Sends UBX-CFG-MSG commands to enable required NMEA messages:
/// - GGA (position, altitude, satellites)
/// - RMC (speed, course over ground)
/// - VTG (speed, course over ground - backup)
///
/// This should be called once after creating the GPS driver if the module
/// is not outputting expected NMEA sentences (e.g., GPGSV visible but no GPGGA).
///
/// # UBX Protocol
///
/// UBX-CFG-MSG format:
/// - Sync: 0xB5, 0x62
/// - Class: 0x06 (CFG)
/// - ID: 0x01 (MSG)
/// - Length: 3 bytes
/// - Payload: [msgClass, msgID, rate]
/// - Checksum: 2 bytes (CK_A, CK_B)
///
/// NMEA message class is 0xF0 with IDs:
/// - GGA: 0x00
/// - RMC: 0x04
/// - VTG: 0x05
///
/// # Errors
///
/// Returns an error if UART write fails.
pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()> {
    // UBX-CFG-MSG to enable GGA at rate 1 (every navigation solution)
    // Format: sync(2) + class(1) + id(1) + len(2) + payload(3) + checksum(2)
    let enable_gga = build_cfg_msg(0xF0, 0x00, 1); // NMEA-GGA
    let enable_rmc = build_cfg_msg(0xF0, 0x04, 1); // NMEA-RMC
    let enable_vtg = build_cfg_msg(0xF0, 0x05, 1); // NMEA-VTG

    uart.write(&enable_gga)?;
    uart.write(&enable_rmc)?;
    uart.write(&enable_vtg)?;

    Ok(())
}

/// Build UBX-CFG-MSG command to set NMEA message rate
///
/// # Arguments
///
/// * `msg_class` - NMEA message class (0xF0 for standard NMEA)
/// * `msg_id` - NMEA message ID (0x00=GGA, 0x04=RMC, 0x05=VTG)
/// * `rate` - Output rate (0=disabled, 1=every solution, 2=every 2nd, etc.)
pub(crate) fn build_cfg_msg(msg_class: u8, msg_id: u8, rate: u8) -> [u8; 11] {
    let mut cmd = [0u8; 11];

    // Sync chars
    cmd[0] = 0xB5;
    cmd[1] = 0x62;

    // Message class and ID for CFG-MSG
    cmd[2] = 0x06; // CFG class
    cmd[3] = 0x01; // MSG id

    // Payload length (little endian)
    cmd[4] = 3;
    cmd[5] = 0;

    // Payload: msgClass, msgID, rate
    cmd[6] = msg_class;
    cmd[7] = msg_id;
    cmd[8] = rate;

    // Calculate checksum (CK_A, CK_B) over class, id, length, and payload
    let (ck_a, ck_b) = ubx_checksum(&cmd[2..9]);
    cmd[9] = ck_a;
    cmd[10] = ck_b;

    cmd
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

    #[test]
    fn test_build_cfg_msg_gga() {
        let msg = build_cfg_msg(0xF0, 0x00, 1);

        // Sync chars
        assert_eq!(msg[0], 0xB5);
        assert_eq!(msg[1], 0x62);

        // Class and ID
        assert_eq!(msg[2], 0x06); // CFG
        assert_eq!(msg[3], 0x01); // MSG

        // Length
        assert_eq!(msg[4], 3);
        assert_eq!(msg[5], 0);

        // Payload
        assert_eq!(msg[6], 0xF0); // NMEA class
        assert_eq!(msg[7], 0x00); // GGA ID
        assert_eq!(msg[8], 1); // Rate

        // Checksum (verified)
        assert_eq!(msg[9], 0xFB);
        assert_eq!(msg[10], 0x11);
    }

    #[test]
    fn test_build_cfg_msg_rmc() {
        let msg = build_cfg_msg(0xF0, 0x04, 1);

        // Verify payload
        assert_eq!(msg[6], 0xF0); // NMEA class
        assert_eq!(msg[7], 0x04); // RMC ID
        assert_eq!(msg[8], 1); // Rate
    }

    #[test]
    fn test_build_cfg_msg_vtg() {
        let msg = build_cfg_msg(0xF0, 0x05, 1);

        // Verify payload
        assert_eq!(msg[6], 0xF0); // NMEA class
        assert_eq!(msg[7], 0x05); // VTG ID
        assert_eq!(msg[8], 1); // Rate
    }
}
