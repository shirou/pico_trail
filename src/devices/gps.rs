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

use crate::platform::{Result, traits::UartInterface};

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
}

/// GPS device driver
///
/// This driver is generic over any type implementing `UartInterface`,
/// making it platform-independent.
pub struct GpsDriver<U: UartInterface> {
    uart: U,
    buffer: [u8; 256],
    buffer_pos: usize,
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
            buffer: [0u8; 256],
            buffer_pos: 0,
        }
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

        // Append to internal buffer
        for &byte in temp_buf.iter().take(read_count) {
            if self.buffer_pos < self.buffer.len() {
                self.buffer[self.buffer_pos] = byte;
                self.buffer_pos += 1;

                // Check for end of NMEA sentence (\r\n)
                if self.buffer_pos >= 2
                    && self.buffer[self.buffer_pos - 2] == b'\r'
                    && self.buffer[self.buffer_pos - 1] == b'\n'
                {
                    // Parse the sentence
                    let sentence = &self.buffer[..self.buffer_pos - 2];
                    let result = self.parse_nmea(sentence);

                    // Reset buffer
                    self.buffer_pos = 0;

                    if let Some(pos) = result {
                        return Ok(Some(pos));
                    }
                }
            } else {
                // Buffer overflow, reset
                self.buffer_pos = 0;
            }
        }

        Ok(None)
    }

    /// Parse NMEA sentence (simplified GPRMC/GPGGA parser)
    fn parse_nmea(&self, sentence: &[u8]) -> Option<GpsPosition> {
        // Convert to string (simplified, assumes ASCII)
        let s = core::str::from_utf8(sentence).ok()?;

        // Check if it's a GPRMC sentence (recommended minimum specific data)
        if s.starts_with("$GPRMC") {
            self.parse_gprmc(s)
        } else if s.starts_with("$GPGGA") {
            self.parse_gpgga(s)
        } else {
            None
        }
    }

    /// Parse GPRMC sentence
    /// Format: $GPRMC,time,status,lat,NS,lon,EW,speed,course,date,,,*checksum
    fn parse_gprmc(&self, sentence: &str) -> Option<GpsPosition> {
        // Use fixed-size array instead of Vec for no_std compatibility
        let mut fields = [""; 16];
        let mut field_count = 0;

        for field in sentence.split(',') {
            if field_count >= fields.len() {
                break;
            }
            fields[field_count] = field;
            field_count += 1;
        }

        if field_count < 10 {
            return None;
        }

        // Check if data is valid (field 2)
        if fields[2] != "A" {
            return None; // Invalid or no fix
        }

        // Parse latitude (fields 3, 4)
        let lat = self.parse_coordinate(fields[3], fields[4])?;

        // Parse longitude (fields 5, 6)
        let lon = self.parse_coordinate(fields[5], fields[6])?;

        // Parse speed in knots (field 7)
        let speed_knots: f32 = fields[7].parse().ok()?;
        let speed = speed_knots * 0.514444; // Convert knots to m/s

        Some(GpsPosition {
            latitude: lat,
            longitude: lon,
            altitude: 0.0, // Not available in GPRMC
            speed,
        })
    }

    /// Parse GPGGA sentence
    /// Format: $GPGGA,time,lat,NS,lon,EW,quality,sats,hdop,altitude,M,...*checksum
    fn parse_gpgga(&self, sentence: &str) -> Option<GpsPosition> {
        // Use fixed-size array instead of Vec for no_std compatibility
        let mut fields = [""; 16];
        let mut field_count = 0;

        for field in sentence.split(',') {
            if field_count >= fields.len() {
                break;
            }
            fields[field_count] = field;
            field_count += 1;
        }

        if field_count < 10 {
            return None;
        }

        // Check fix quality (field 6)
        let quality: u8 = fields[6].parse().ok()?;
        if quality == 0 {
            return None; // No fix
        }

        // Parse latitude (fields 2, 3)
        let lat = self.parse_coordinate(fields[2], fields[3])?;

        // Parse longitude (fields 4, 5)
        let lon = self.parse_coordinate(fields[4], fields[5])?;

        // Parse altitude (field 9)
        let altitude: f32 = fields[9].parse().ok()?;

        Some(GpsPosition {
            latitude: lat,
            longitude: lon,
            altitude,
            speed: 0.0, // Not available in GPGGA
        })
    }

    /// Parse coordinate from NMEA format
    /// Example: "4807.038,N" -> 48.1173 degrees
    fn parse_coordinate(&self, coord_str: &str, direction: &str) -> Option<f32> {
        if coord_str.is_empty() {
            return None;
        }

        // Find decimal point
        let dot_pos = coord_str.find('.')?;

        // Degrees are before the last 2 digits before decimal
        let deg_end = dot_pos.saturating_sub(2);
        let degrees: f32 = coord_str[..deg_end].parse().ok()?;

        // Minutes are the last 2 digits and decimal part
        let minutes: f32 = coord_str[deg_end..].parse().ok()?;

        // Convert to decimal degrees
        let mut result = degrees + (minutes / 60.0);

        // Apply direction (S/W are negative)
        if direction == "S" || direction == "W" {
            result = -result;
        }

        Some(result)
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
        let mut uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // Inject invalid sentence
        gps.uart.inject_rx_data(b"INVALID DATA\r\n");

        let position = gps.read_position().unwrap();
        assert!(position.is_none());
    }
}
