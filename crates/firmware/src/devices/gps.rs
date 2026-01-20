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

pub mod init;

/// GPS latitude offset for privacy (from build-time environment variable)
/// Added to all latitude values to hide actual location in public displays
const GPS_LAT_OFFSET: f32 = {
    // build.rs parses the environment variable and outputs the parsed f32 value
    const S: &str = env!("GPS_LAT_OFFSET");
    const_parse_f32(S)
};

/// GPS longitude offset for privacy (from build-time environment variable)
/// Added to all longitude values to hide actual location in public displays
const GPS_LON_OFFSET: f32 = {
    const S: &str = env!("GPS_LON_OFFSET");
    const_parse_f32(S)
};

/// Parse f32 from string at compile time
/// Handles optional sign, integer part, and optional decimal part
const fn const_parse_f32(s: &str) -> f32 {
    let bytes = s.as_bytes();
    if bytes.is_empty() {
        return 0.0;
    }

    let mut i = 0;
    let negative = bytes[0] == b'-';
    if negative || bytes[0] == b'+' {
        i += 1;
    }

    let mut int_part: i64 = 0;
    while i < bytes.len() && bytes[i] != b'.' {
        if bytes[i] < b'0' || bytes[i] > b'9' {
            return 0.0; // Invalid character
        }
        int_part = int_part * 10 + (bytes[i] - b'0') as i64;
        i += 1;
    }

    let mut frac_part: i64 = 0;
    let mut frac_digits: u32 = 0;
    if i < bytes.len() && bytes[i] == b'.' {
        i += 1;
        while i < bytes.len() {
            if bytes[i] < b'0' || bytes[i] > b'9' {
                return 0.0; // Invalid character
            }
            frac_part = frac_part * 10 + (bytes[i] - b'0') as i64;
            frac_digits += 1;
            i += 1;
        }
    }

    let mut divisor: i64 = 1;
    let mut d = 0;
    while d < frac_digits {
        divisor *= 10;
        d += 1;
    }

    let value = int_part as f64 + (frac_part as f64 / divisor as f64);
    if negative {
        -(value as f32)
    } else {
        value as f32
    }
}

/// GPS fix type
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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

/// Internal state for combining GGA and RMC data
#[derive(Debug, Clone, Copy, Default)]
struct GpsInternalState {
    /// Latitude from GGA (degrees)
    latitude: Option<f32>,
    /// Longitude from GGA (degrees)
    longitude: Option<f32>,
    /// Altitude from GGA (meters)
    altitude: Option<f32>,
    /// Satellites from GGA
    satellites: Option<u8>,
    /// Speed from RMC (m/s)
    speed: Option<f32>,
    /// Course over ground from RMC (degrees)
    course_over_ground: Option<f32>,
    /// Fix type derived from GGA
    fix_type: Option<GpsFixType>,
}

impl GpsInternalState {
    /// Check if we have minimum required data (position from GGA)
    fn has_valid_position(&self) -> bool {
        self.latitude.is_some() && self.longitude.is_some() && self.fix_type.is_some()
    }

    /// Convert to GpsPosition if valid
    fn to_position(self) -> Option<GpsPosition> {
        if !self.has_valid_position() {
            return None;
        }

        // Pass through COG as-is from RMC sentence
        // Note: COG reliability depends on speed - consumers should check speed
        // before using COG for navigation (typically valid when speed >= 0.5 m/s)
        // Apply privacy offset to latitude/longitude (configured via GPS_LAT_OFFSET/GPS_LON_OFFSET env vars)
        Some(GpsPosition {
            latitude: self.latitude.unwrap_or(0.0) + GPS_LAT_OFFSET,
            longitude: self.longitude.unwrap_or(0.0) + GPS_LON_OFFSET,
            altitude: self.altitude.unwrap_or(0.0),
            speed: self.speed.unwrap_or(0.0),
            course_over_ground: self.course_over_ground,
            fix_type: self.fix_type.unwrap_or(GpsFixType::NoFix),
            satellites: self.satellites.unwrap_or(0),
        })
    }

    /// Update from GGA sentence
    fn update_from_gga(&mut self, gga: &nmea0183::GGA) {
        self.latitude = Some(gga.latitude.as_f64() as f32);
        self.longitude = Some(gga.longitude.as_f64() as f32);
        self.altitude = gga.altitude.as_ref().map(|a| a.meters);
        self.satellites = Some(gga.sat_in_use);

        // Determine fix type based on altitude availability
        self.fix_type = Some(
            if gga
                .altitude
                .as_ref()
                .map(|a| a.meters.abs() > 0.01)
                .unwrap_or(false)
            {
                GpsFixType::Fix3D
            } else {
                GpsFixType::Fix2D
            },
        );
    }

    /// Update from RMC sentence
    fn update_from_rmc(&mut self, rmc: &nmea0183::RMC) {
        // Speed in knots -> m/s (1 knot = 0.514444 m/s)
        self.speed = Some(rmc.speed.as_knots() * 0.514_444);

        // Course over ground - update when present in RMC
        // Note: GPS modules typically only send COG when speed > ~0.5 knots
        if let Some(course) = &rmc.course {
            self.course_over_ground = Some(course.degrees);
        }
    }

    /// Update from VTG sentence (backup source for COG)
    fn update_from_vtg(&mut self, vtg: &nmea0183::VTG) {
        // Speed in knots -> m/s (1 knot = 0.514444 m/s)
        self.speed = Some(vtg.speed.as_knots() * 0.514_444);

        // Course over ground from VTG
        if let Some(course) = &vtg.course {
            self.course_over_ground = Some(course.degrees);
        }
    }
}

/// GPS device driver
///
/// This driver is generic over any type implementing `UartInterface`,
/// making it platform-independent.
///
/// The driver combines data from multiple NMEA sentence types:
/// - **GGA**: Position (lat, lon, alt) and satellite count
/// - **RMC**: Speed and Course Over Ground (COG)
///
/// Both sentence types are parsed and merged to provide complete GPS data.
pub struct GpsDriver<U: UartInterface> {
    uart: U,
    parser: Parser,
    /// Internal state for combining GGA and RMC data
    state: GpsInternalState,
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
            state: GpsInternalState::default(),
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

    /// Get mutable reference to UART interface
    ///
    /// Used for direct UART access, primarily for vendor-specific
    /// initialization commands.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use pico_trail::devices::gps::{GpsDriver, init::ublox};
    ///
    /// let mut gps = GpsDriver::new(uart);
    /// ublox::initialize(gps.uart_mut())?;
    /// ```
    pub fn uart_mut(&mut self) -> &mut U {
        &mut self.uart
    }

    /// Read GPS position if available
    ///
    /// Returns `Some(GpsPosition)` if valid NMEA data has been parsed and
    /// a complete position is available. The driver combines data from:
    /// - **GGA**: Position (lat, lon, alt) and satellite count
    /// - **RMC**: Speed and Course Over Ground (COG)
    /// - **VTG**: Speed and Course Over Ground (backup source)
    ///
    /// Position is returned when GGA data is available. RMC/VTG data (speed, COG)
    /// is merged when received.
    ///
    /// # Errors
    ///
    /// Returns an error if UART communication fails.
    pub fn read_position(&mut self) -> Result<Option<GpsPosition>> {
        // Read available data from UART
        let mut temp_buf = [0u8; 64];
        let read_count = self.uart.read(&mut temp_buf)?;

        if read_count == 0 {
            // No new data, return current position if valid
            return Ok(self.state.to_position());
        }

        let mut position_updated = false;

        // Process each byte through the NMEA parser
        for &byte in temp_buf.iter().take(read_count) {
            if let Some(result) = self.parser.parse_from_byte(byte) {
                match result {
                    // GGA sentence - position, altitude, satellite count
                    Ok(ParseResult::GGA(Some(gga))) => {
                        self.state.update_from_gga(&gga);
                        position_updated = true;
                    }
                    // RMC sentence - speed and course over ground
                    Ok(ParseResult::RMC(Some(rmc))) => {
                        self.state.update_from_rmc(&rmc);
                        position_updated = true;
                    }
                    // VTG sentence - speed and course over ground (backup)
                    Ok(ParseResult::VTG(Some(vtg))) => {
                        self.state.update_from_vtg(&vtg);
                        position_updated = true;
                    }
                    // Valid sentence but no data, or other sentence types - ignore
                    _ => {}
                }
            }
        }

        // Return position if we have valid data and something was updated
        if position_updated {
            Ok(self.state.to_position())
        } else {
            Ok(None)
        }
    }

    /// Get current GPS position from cached state
    ///
    /// Returns the current position without reading from UART.
    /// Useful when you need the last known position without blocking.
    pub fn current_position(&self) -> Option<GpsPosition> {
        self.state.to_position()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockUart;
    use crate::platform::traits::UartConfig;

    #[test]
    fn test_gps_rmc_only_no_position() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // RMC alone doesn't provide full position (no satellite count, no altitude)
        // RMC updates speed/COG but position requires GGA first
        let nmea = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        // RMC alone should not return position (GGA required for valid position)
        let position = gps.read_position().unwrap();
        assert!(position.is_none());
    }

    #[test]
    fn test_gps_parse_gpgga() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // Inject GPGGA sentence
        let nmea = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");
        assert!((position.latitude - 48.1173).abs() < 0.001);
        assert!((position.longitude - 11.516_666).abs() < 0.001);
        assert_eq!(position.satellites, 8);
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
        gps.uart_mut().inject_rx_data(b"INVALID DATA\r\n");

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
    fn test_gps_parse_gpgga_no_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // GPGGA sentence does not contain COG
        let nmea = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(nmea);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");
        // GPGGA does not provide COG, so it should be None (no RMC received yet)
        assert!(position.course_over_ground.is_none());
        // Speed should be 0 without RMC
        assert_eq!(position.speed, 0.0);
    }

    #[test]
    fn test_gps_gga_then_rmc_combined() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // First inject GGA for position and satellite count
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(gga);
        let _ = read_gps_position_complete(&mut gps);

        // Then inject RMC for speed and COG
        // Speed: 22.4 knots = ~11.52 m/s, COG: 84.4 degrees
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        gps.uart_mut().inject_rx_data(rmc);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");

        // Verify position from GGA
        assert!((position.latitude - 48.1173).abs() < 0.001);
        assert!((position.longitude - 11.516_666).abs() < 0.001);
        assert!((position.altitude - 545.4).abs() < 0.1);
        assert_eq!(position.satellites, 8);

        // Verify speed and COG from RMC
        // 22.4 knots * 0.514444 = ~11.52 m/s
        assert!((position.speed - 11.52).abs() < 0.1);
        // COG should be present since speed > 0.5 m/s
        assert!(position.course_over_ground.is_some());
        assert!((position.course_over_ground.unwrap() - 84.4).abs() < 0.1);
    }

    #[test]
    fn test_gps_rmc_low_speed_has_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // First inject GGA for position
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(gga);
        let _ = read_gps_position_complete(&mut gps);

        // RMC with very low speed (0.5 knots = ~0.26 m/s)
        // COG is still provided by GPS even at low speed
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,000.5,084.4,230394,003.1,W*6B\r\n";
        gps.uart_mut().inject_rx_data(rmc);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");

        // Speed should be ~0.26 m/s
        assert!((position.speed - 0.257).abs() < 0.1);
        // COG is passed through from RMC (consumers should check speed for reliability)
        assert!(position.course_over_ground.is_some());
        assert!((position.course_over_ground.unwrap() - 84.4).abs() < 0.1);
    }

    #[test]
    fn test_gps_gnrmc_provides_speed_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // First inject GNGGA for position
        let gga = b"$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
        gps.uart_mut().inject_rx_data(gga);
        let _ = read_gps_position_complete(&mut gps);

        // Then inject GNRMC for speed/COG
        let rmc = b"$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*74\r\n";
        gps.uart_mut().inject_rx_data(rmc);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");

        // Verify speed and COG from GNRMC
        assert!((position.speed - 11.52).abs() < 0.1);
        assert!(position.course_over_ground.is_some());
        assert!((position.course_over_ground.unwrap() - 84.4).abs() < 0.1);
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

    #[test]
    fn test_gps_current_position() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // Initially no position
        assert!(gps.current_position().is_none());

        // Inject GGA
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(gga);
        let _ = read_gps_position_complete(&mut gps);

        // Now current_position should return the cached position
        let position = gps.current_position().expect("Expected cached position");
        assert!((position.latitude - 48.1173).abs() < 0.001);
    }

    #[test]
    fn test_gps_vtg_provides_speed_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // First inject GGA for position
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(gga);
        let _ = read_gps_position_complete(&mut gps);

        // Then inject VTG for speed/COG (backup source)
        // VTG: course 89.0 degrees, speed 15.2 knots
        let vtg = b"$GPVTG,089.0,T,,,15.2,N,,,A*12\r\n";
        gps.uart_mut().inject_rx_data(vtg);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");

        // Verify speed from VTG (15.2 knots * 0.514444 = ~7.82 m/s)
        assert!((position.speed - 7.82).abs() < 0.1);
        // Verify COG from VTG
        assert!(position.course_over_ground.is_some());
        assert!((position.course_over_ground.unwrap() - 89.0).abs() < 0.1);
    }

    #[test]
    fn test_gps_vtg_without_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // First inject GGA for position
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(gga);
        let _ = read_gps_position_complete(&mut gps);

        // VTG with empty COG field (stationary)
        // Note: Some GPS modules send VTG with empty COG when stationary
        let vtg = b"$GPVTG,,T,,,0.0,N,,,A*23\r\n";
        gps.uart_mut().inject_rx_data(vtg);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");

        // Speed should be 0
        assert!((position.speed - 0.0).abs() < 0.01);
        // COG should be None (empty in VTG)
        assert!(position.course_over_ground.is_none());
    }

    #[test]
    fn test_gps_rmc_then_vtg_updates_cog() {
        let uart = MockUart::new(UartConfig::default());
        let mut gps = GpsDriver::new(uart);

        // First inject GGA for position
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        gps.uart_mut().inject_rx_data(gga);
        let _ = read_gps_position_complete(&mut gps);

        // RMC with COG
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        gps.uart_mut().inject_rx_data(rmc);
        let _ = read_gps_position_complete(&mut gps);

        // VTG with different COG should update it
        let vtg = b"$GPVTG,095.0,T,,,20.0,N,,,A*1B\r\n";
        gps.uart_mut().inject_rx_data(vtg);

        let position = read_gps_position_complete(&mut gps).expect("Expected GPS position");

        // VTG should have updated both speed and COG
        assert!((position.speed - 10.29).abs() < 0.1); // 20.0 knots * 0.514444
        assert!(position.course_over_ground.is_some());
        assert!((position.course_over_ground.unwrap() - 95.0).abs() < 0.1);
    }
}
