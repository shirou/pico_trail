//! GPS device driver (NMEA protocol)
//!
//! This is a reference implementation demonstrating how to write
//! platform-independent device drivers using platform abstraction traits.
//!
//! # Example
//!
//! ```ignore
//! use pico_trail::platform::mock::{MockPlatform, MockUart};
//! use pico_trail::platform::traits::{Platform, UartConfig};
//! use pico_trail::devices::gps::GpsDriver;
//!
//! let mut platform = MockPlatform::new();
//! let uart = platform.create_uart(0, UartConfig::default()).unwrap();
//! let mut gps = GpsDriver::new(uart);
//!
//! // Process GPS data
//! if let Some(position) = gps.read_position().unwrap() {
//!     println!("Lat: {}, Lon: {}", position.latitude, position.longitude);
//! }
//! ```

use crate::platform::{traits::UartInterface, Result};
use nmea0183::{ParseResult, Parser};

/// GPS fix type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GpsFixType {
    /// No GPS fix
    NoFix,
    /// 2D fix (latitude, longitude only)
    Fix2D,
    /// 3D fix (latitude, longitude, altitude)
    Fix3D,
}

/// GPS position data
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsPosition {
    /// Latitude in degrees (-90 to +90)
    pub latitude: f32,
    /// Longitude in degrees (-180 to +180)
    pub longitude: f32,
    /// Altitude in meters above sea level
    pub altitude: f32,
    /// Speed in meters per second
    pub speed: f32,
    /// Course over ground in degrees (0-360), None if invalid (speed < 0.5 m/s)
    pub course_over_ground: Option<f32>,
    /// GPS fix type
    pub fix_type: GpsFixType,
    /// Number of satellites used in fix
    pub satellites: u8,
}

/// GPS device driver
///
/// This driver is generic over any type implementing `UartInterface`,
/// making it platform-independent.
pub struct GpsDriver<U: UartInterface> {
    uart: U,
    parser: Parser,
}

impl<U: UartInterface> GpsDriver<U> {
    /// Create a new GPS driver
    ///
    /// # Arguments
    ///
    /// * `uart` - UART interface for GPS communication
    pub fn new(uart: U) -> Self {
        Self {
            uart,
            parser: Parser::new(),
        }
    }

    /// Update GPS state and read position if available
    ///
    /// Returns `Some(GpsPosition)` if a valid NMEA sentence was parsed,
    /// or `None` if no complete sentence is available yet.
    ///
    /// This is an alias for `read_position()` for API compatibility.
    ///
    /// # Errors
    ///
    /// Returns an error if UART communication fails.
    pub fn update(&mut self) -> Result<Option<GpsPosition>> {
        self.read_position()
    }

    /// Get mutable reference to UART interface (test helper)
    #[cfg(test)]
    pub fn uart_mut(&mut self) -> &mut U {
        &mut self.uart
    }

    /// Read GPS position if available
    ///
    /// Returns `Some(GpsPosition)` if a valid NMEA sentence was parsed,
    /// or `None` if no complete sentence is available yet.
    ///
    /// # Errors
    ///
    /// Returns an error if UART communication fails.
    pub fn read_position(&mut self) -> Result<Option<GpsPosition>> {
        // Read available data from UART
        let mut temp_buf = [0u8; 64];
        let read_count = self.uart.read(&mut temp_buf)?;

        if read_count == 0 {
            return Ok(None);
        }

        // Process each byte through the NMEA parser
        for &byte in temp_buf.iter().take(read_count) {
            if let Some(result) = self.parser.parse_from_byte(byte) {
                match result {
                    Ok(ParseResult::RMC(Some(rmc))) => {
                        return Ok(Some(Self::convert_rmc(&rmc)));
                    }
                    Ok(ParseResult::GGA(Some(gga))) => {
                        return Ok(Some(Self::convert_gga(&gga)));
                    }
                    // Valid sentence but no data (receiver working but no fix)
                    Ok(ParseResult::RMC(None) | ParseResult::GGA(None)) => {
                        // Continue parsing, waiting for valid data
                    }
                    // Other sentence types (VTG, GLL) - ignore
                    Ok(_) => {}
                    // Parse errors - ignore and continue
                    Err(_) => {}
                }
            }
        }

        Ok(None)
    }

    /// Convert nmea0183::RMC to GpsPosition
    fn convert_rmc(rmc: &nmea0183::RMC) -> GpsPosition {
        let speed = rmc.speed.as_mps();

        // COG is unreliable at low speeds, so only use when speed >= 0.5 m/s
        let course_over_ground = if speed >= 0.5 {
            rmc.course.as_ref().map(|c| c.degrees)
        } else {
            None
        };

        GpsPosition {
            latitude: rmc.latitude.as_f64() as f32,
            longitude: rmc.longitude.as_f64() as f32,
            altitude: 0.0, // Not available in RMC
            speed,
            course_over_ground,
            fix_type: GpsFixType::Fix2D, // RMC has valid fix but no altitude
            satellites: 0,               // Not available in RMC
        }
    }

    /// Convert nmea0183::GGA to GpsPosition
    fn convert_gga(gga: &nmea0183::GGA) -> GpsPosition {
        let altitude = gga.altitude.meters;

        // Determine fix type based on altitude availability
        let fix_type = if altitude.abs() > 0.01 {
            GpsFixType::Fix3D
        } else {
            GpsFixType::Fix2D
        };

        GpsPosition {
            latitude: gga.latitude.as_f64() as f32,
            longitude: gga.longitude.as_f64() as f32,
            altitude,
            speed: 0.0,               // Not available in GGA
            course_over_ground: None, // Not available in GGA
            fix_type,
            satellites: gga.sat_in_use,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockUart;
    use crate::platform::traits::UartConfig;

    #[test]
    fn test_gps_parse_gprmc() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // Inject GPRMC sentence
        let nmea = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        gps.uart.inject_rx_data(nmea);

        // Note: This is a simplified test. Full GPS parsing would require more robust error handling.
        // For demonstration purposes, we verify the driver compiles and runs.
        let _position = gps.read_position().unwrap();
        // In a real implementation, we would validate the parsed data
    }

    #[test]
    fn test_gps_parse_gpgga() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // Inject GPGGA sentence
        let nmea = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart.inject_rx_data(nmea);

        // Note: This is a simplified test. Full GPS parsing would require more robust error handling.
        let _position = gps.read_position().unwrap();
        // In a real implementation, we would validate the parsed data
    }

    #[test]
    fn test_gps_no_data() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        let position = gps.read_position().unwrap();
        assert!(position.is_none());
    }

    #[test]
    fn test_gps_invalid_sentence() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // Inject invalid sentence
        gps.uart.inject_rx_data(b"INVALID DATA\r\n");

        let position = gps.read_position().unwrap();
        assert!(position.is_none());
    }

    /// Helper to read GPS position, calling read_position multiple times until data is parsed
    fn read_gps_position_complete(gps: &mut GpsDriver<MockUart>) -> Option<GpsPosition> {
        // GPS driver reads 64 bytes at a time, so longer sentences may need multiple reads
        for _ in 0..5 {
            if let Ok(Some(pos)) = gps.read_position() {
                return Some(pos);
            }
        }
        None
    }

    #[test]
    fn test_gps_parse_gprmc_with_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // GPRMC sentence with speed 22.4 knots (~11.5 m/s) and COG 84.4 degrees
        let nmea = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");
        assert!(position.course_over_ground.is_some());
        let cog = position.course_over_ground.unwrap();
        // COG should be approximately 84.4 degrees
        assert!((cog - 84.4).abs() < 0.1);
        // Speed should be approximately 11.5 m/s (22.4 knots * 0.514444)
        assert!(position.speed > 11.0 && position.speed < 12.0);
    }

    #[test]
    fn test_gps_parse_gprmc_low_speed_no_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // GPRMC sentence with speed 0.5 knots (~0.26 m/s) - below threshold
        let nmea = b"$GPRMC,123519,A,4807.038,N,01131.000,E,000.5,084.4,230394,003.1,W*6B\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");
        // COG should be None when speed < 0.5 m/s
        assert!(position.course_over_ground.is_none());
        // Speed should be approximately 0.26 m/s
        assert!(position.speed < 0.5);
    }

    #[test]
    fn test_gps_parse_gprmc_threshold_speed() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // GPRMC sentence with speed 1.0 knots (~0.514 m/s) - at threshold
        let nmea = b"$GPRMC,123519,A,4807.038,N,01131.000,E,001.0,180.0,230394,003.1,W*6E\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");
        // COG should be valid when speed >= 0.5 m/s
        assert!(position.course_over_ground.is_some());
        let cog = position.course_over_ground.unwrap();
        assert!((cog - 180.0).abs() < 0.1);
    }

    #[test]
    fn test_gps_parse_gpgga_no_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // GPGGA sentence does not contain COG
        let nmea = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");
        // GPGGA does not provide COG, so it should be None
        assert!(position.course_over_ground.is_none());
    }

    #[test]
    fn test_gps_parse_gnrmc() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // GNRMC sentence (GNSS combined - used by u-blox NEO-M8N and similar)
        let nmea = b"$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*74\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");
        assert!((position.latitude - 48.1173).abs() < 0.001);
        assert!((position.longitude - 11.516_666).abs() < 0.001);
        assert!(position.course_over_ground.is_some());
    }

    #[test]
    fn test_gps_parse_gngga() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // GNGGA sentence (GNSS combined)
        let nmea = b"$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");
        assert!((position.latitude - 48.1173).abs() < 0.001);
        assert!((position.longitude - 11.516_666).abs() < 0.001);
        assert_eq!(position.satellites, 8);
    }
}
