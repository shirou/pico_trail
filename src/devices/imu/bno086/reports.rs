//! BNO086 Sensor Report Parsing
//!
//! This module handles parsing of BNO086-specific sensor reports from SHTP packets.
//! The BNO086 uses fixed-point representation (Q-format) for most values.
//!
//! # Report Types
//!
//! - Rotation Vector (0x05): Quaternion orientation with accuracy
//! - Game Rotation Vector (0x08): Quaternion without magnetometer
//! - Gyroscope Calibrated (0x02): Angular velocity in rad/s
//! - Product ID Response (0xF8): Device identification
//!
//! # Fixed-Point Formats
//!
//! - Q14: Quaternion components (scale = 1/16384)
//! - Q12: Accuracy in radians (scale = 1/4096)
//! - Q9: Gyroscope angular velocity (scale = 1/512)
//!
//! # Coordinate Frame
//!
//! BNO086 outputs in Android-style coordinate system (X=right, Y=up, Z=out).
//! For NED (North-East-Down) frame used in flight control, coordinate conversion
//! is required in the AHRS abstraction layer.
//!
//! # References
//!
//! - BNO080/BNO085/BNO086 Datasheet - Section 6.5: Sensor Reports
//! - SH-2 Reference Manual - Gyroscope Calibrated Report (0x02)

use nalgebra::{Quaternion, Vector3};

/// BNO086 Report IDs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ReportId {
    /// Product ID Response
    ProductIdResponse = 0xF8,
    /// Rotation Vector (quaternion with magnetometer)
    RotationVector = 0x05,
    /// Game Rotation Vector (quaternion without magnetometer)
    GameRotationVector = 0x08,
    /// Gyroscope Calibrated
    GyroscopeCalibrated = 0x02,
    /// Accelerometer
    Accelerometer = 0x01,
    /// Command Response
    CommandResponse = 0xF1,
    /// Get Feature Response
    GetFeatureResponse = 0xFC,
}

impl ReportId {
    /// Convert from raw byte
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0xF8 => Some(Self::ProductIdResponse),
            0x05 => Some(Self::RotationVector),
            0x08 => Some(Self::GameRotationVector),
            0x02 => Some(Self::GyroscopeCalibrated),
            0x01 => Some(Self::Accelerometer),
            0xF1 => Some(Self::CommandResponse),
            0xFC => Some(Self::GetFeatureResponse),
            _ => None,
        }
    }
}

/// Q14 fixed-point scale factor (1/16384)
const Q14_SCALE: f32 = 1.0 / 16384.0;

/// Q12 fixed-point scale factor (1/4096)
const Q12_SCALE: f32 = 1.0 / 4096.0;

/// Q9 fixed-point scale factor (1/512) - used for gyroscope data
const Q9_SCALE: f32 = 1.0 / 512.0;

/// BNO086 Rotation Vector Report
///
/// Parsed from Report ID 0x05. Contains quaternion orientation and accuracy estimate.
///
/// # Memory Layout (14 bytes total after report header)
///
/// ```text
/// Offset  Size  Description
/// 0       1     Report ID (0x05)
/// 1       1     Sequence number
/// 2       1     Status
/// 3       1     Delay (LSB)
/// 4       1     Delay (MSB)
/// 5-6     2     Q_i (Q14 fixed-point)
/// 7-8     2     Q_j (Q14 fixed-point)
/// 9-10    2     Q_k (Q14 fixed-point)
/// 11-12   2     Q_real (Q14 fixed-point)
/// 13-14   2     Accuracy (Q12 fixed-point, radians)
/// ```
#[derive(Debug, Clone, Copy, Default)]
pub struct RotationVectorReport {
    /// Sequence number for this report
    pub sequence: u8,
    /// Status byte (accuracy bits in lower 2 bits)
    pub status: u8,
    /// Quaternion i component (Q14 fixed-point)
    pub q_i: i16,
    /// Quaternion j component (Q14 fixed-point)
    pub q_j: i16,
    /// Quaternion k component (Q14 fixed-point)
    pub q_k: i16,
    /// Quaternion real/w component (Q14 fixed-point)
    pub q_real: i16,
    /// Accuracy estimate (Q12 fixed-point, radians)
    pub accuracy: i16,
}

impl RotationVectorReport {
    /// Minimum payload size for a valid Rotation Vector report (0x05)
    /// With offset 4 base: header(4) + quat(8) + accuracy(2) = 14 bytes
    pub const MIN_PAYLOAD_SIZE: usize = 14;

    /// Minimum payload size for Game Rotation Vector report (0x08, no accuracy)
    /// With offset 4 base: header(4) + quat(8) = 12 bytes
    pub const GAME_RV_MIN_SIZE: usize = 12;

    /// Parse from SHTP payload bytes (Rotation Vector only)
    ///
    /// # Arguments
    ///
    /// * `payload` - Raw payload bytes (must be at least 14 bytes)
    ///
    /// # Returns
    ///
    /// * `Some(report)` - Successfully parsed report
    /// * `None` - Payload too small or wrong report ID
    pub fn parse(payload: &[u8]) -> Option<Self> {
        if payload.len() < Self::MIN_PAYLOAD_SIZE {
            return None;
        }

        // Verify report ID
        if payload[0] != ReportId::RotationVector as u8 {
            return None;
        }

        Self::parse_inner(payload, ReportId::RotationVector as u8)
    }

    /// Parse from SHTP payload bytes (any quaternion report: 0x05 or 0x08)
    ///
    /// Accepts both Rotation Vector (0x05) and Game Rotation Vector (0x08)
    pub fn parse_any(payload: &[u8]) -> Option<Self> {
        if payload.is_empty() {
            return None;
        }

        let report_id = payload[0];

        // Check minimum size based on report type
        let min_size = if report_id == ReportId::RotationVector as u8 {
            Self::MIN_PAYLOAD_SIZE // 15 bytes for 0x05
        } else if report_id == ReportId::GameRotationVector as u8 {
            Self::GAME_RV_MIN_SIZE // 13 bytes for 0x08
        } else {
            return None;
        };

        if payload.len() < min_size {
            return None;
        }

        Self::parse_inner(payload, report_id)
    }

    /// Internal parse helper
    fn parse_inner(payload: &[u8], report_id: u8) -> Option<Self> {
        // SH-2 Reference Manual report structure (CEVA offset 4 base):
        // Byte 0: Report ID
        // Byte 1: Sequence number
        // Byte 2: Status
        // Byte 3: Delay
        // Byte 4-5: Q_i (16-bit LE, Q14)
        // Byte 6-7: Q_j (16-bit LE, Q14)
        // Byte 8-9: Q_k (16-bit LE, Q14)
        // Byte 10-11: Q_real (16-bit LE, Q14)
        // Byte 12-13: Accuracy (for 0x05 only)
        //
        // Reference: CEVA SH-2 driver (sh2_util.c read16: p[0] | p[1]<<8 = little-endian)

        let q_i = i16::from_le_bytes([payload[4], payload[5]]);
        let q_j = i16::from_le_bytes([payload[6], payload[7]]);
        let q_k = i16::from_le_bytes([payload[8], payload[9]]);
        let q_real = i16::from_le_bytes([payload[10], payload[11]]);

        // Validate quaternion components are in valid range for unit quaternion
        // Q14 format: valid range is approximately -16384 to +16384 (-1.0 to +1.0)
        const MAX_Q14: i16 = 17000; // ~1.04, slight margin
        if q_i.abs() > MAX_Q14
            || q_j.abs() > MAX_Q14
            || q_k.abs() > MAX_Q14
            || q_real.abs() > MAX_Q14
        {
            return None;
        }

        // Accuracy only present in Rotation Vector (0x05), at bytes 12-13
        // (with offset 4 base: 4+8=12 for accuracy start)
        let accuracy = if report_id == ReportId::RotationVector as u8 && payload.len() >= 14 {
            i16::from_le_bytes([payload[12], payload[13]])
        } else {
            0 // Game Rotation Vector has no accuracy
        };

        Some(Self {
            sequence: payload[1],
            status: payload[2],
            q_i,
            q_j,
            q_k,
            q_real,
            accuracy,
        })
    }

    /// Convert to nalgebra Quaternion
    ///
    /// Returns a unit quaternion (w, x, y, z) representing orientation.
    /// The BNO086 outputs quaternions in (i, j, k, real) order, which maps to
    /// nalgebra's (w, x, y, z) as (real, i, j, k).
    pub fn to_quaternion(&self) -> Quaternion<f32> {
        Quaternion::new(
            self.q_real as f32 * Q14_SCALE, // w
            self.q_i as f32 * Q14_SCALE,    // x
            self.q_j as f32 * Q14_SCALE,    // y
            self.q_k as f32 * Q14_SCALE,    // z
        )
    }

    /// Convert accuracy to radians
    ///
    /// Returns the estimated accuracy of the quaternion measurement.
    /// Lower values indicate higher accuracy.
    pub fn accuracy_radians(&self) -> f32 {
        self.accuracy as f32 * Q12_SCALE
    }

    /// Get status accuracy level (0-3)
    ///
    /// - 0: Unreliable
    /// - 1: Low accuracy
    /// - 2: Medium accuracy
    /// - 3: High accuracy
    pub fn accuracy_status(&self) -> u8 {
        self.status & 0x03
    }
}

/// BNO086 Gyroscope Calibrated Report
///
/// Parsed from Report ID 0x02. Contains calibrated angular velocity in rad/s.
///
/// # Memory Layout (10 bytes total)
///
/// ```text
/// Offset  Size  Description
/// 0       1     Report ID (0x02)
/// 1       1     Sequence number
/// 2       1     Status (accuracy in bits 1:0)
/// 3       1     Delay
/// 4-5     2     Gyro X (Q9 fixed-point, rad/s)
/// 6-7     2     Gyro Y (Q9 fixed-point, rad/s)
/// 8-9     2     Gyro Z (Q9 fixed-point, rad/s)
/// ```
///
/// # Coordinate Frame
///
/// BNO086 outputs in Android-style coordinate system.
/// For NED frame, conversion is applied in the AHRS layer.
#[derive(Debug, Clone, Copy, Default)]
pub struct GyroscopeReport {
    /// Sequence number for this report
    pub sequence: u8,
    /// Status byte (accuracy bits in lower 2 bits)
    pub status: u8,
    /// Angular velocity X (Q9 fixed-point, rad/s)
    pub gyro_x: i16,
    /// Angular velocity Y (Q9 fixed-point, rad/s)
    pub gyro_y: i16,
    /// Angular velocity Z (Q9 fixed-point, rad/s)
    pub gyro_z: i16,
}

impl GyroscopeReport {
    /// Minimum payload size for a valid Gyroscope Calibrated report
    /// header(4) + gyro(6) = 10 bytes
    pub const MIN_PAYLOAD_SIZE: usize = 10;

    /// Parse from SHTP payload bytes
    ///
    /// # Arguments
    ///
    /// * `payload` - Raw payload bytes (must be at least 10 bytes)
    ///
    /// # Returns
    ///
    /// * `Some(report)` - Successfully parsed report
    /// * `None` - Payload too small or wrong report ID
    pub fn parse(payload: &[u8]) -> Option<Self> {
        if payload.len() < Self::MIN_PAYLOAD_SIZE {
            return None;
        }

        // Verify report ID
        if payload[0] != ReportId::GyroscopeCalibrated as u8 {
            return None;
        }

        // SH-2 Reference Manual report structure:
        // Byte 0: Report ID (0x02)
        // Byte 1: Sequence number
        // Byte 2: Status
        // Byte 3: Delay
        // Byte 4-5: Gyro X (16-bit LE, Q9)
        // Byte 6-7: Gyro Y (16-bit LE, Q9)
        // Byte 8-9: Gyro Z (16-bit LE, Q9)

        let gyro_x = i16::from_le_bytes([payload[4], payload[5]]);
        let gyro_y = i16::from_le_bytes([payload[6], payload[7]]);
        let gyro_z = i16::from_le_bytes([payload[8], payload[9]]);

        Some(Self {
            sequence: payload[1],
            status: payload[2],
            gyro_x,
            gyro_y,
            gyro_z,
        })
    }

    /// Convert to angular velocity vector (rad/s)
    ///
    /// Returns angular velocity in the sensor's coordinate frame (Android-style).
    /// For NED frame conversion, use `to_angular_velocity_ned()`.
    pub fn to_angular_velocity(&self) -> Vector3<f32> {
        Vector3::new(
            self.gyro_x as f32 * Q9_SCALE,
            self.gyro_y as f32 * Q9_SCALE,
            self.gyro_z as f32 * Q9_SCALE,
        )
    }

    /// Convert to angular velocity vector in NED frame (rad/s)
    ///
    /// Applies coordinate transformation from BNO086's Android-style frame
    /// to NED (North-East-Down) frame used in flight control.
    ///
    /// BNO086 (Android): X=right, Y=up, Z=out (of screen)
    /// NED: X=north/forward, Y=east/right, Z=down
    ///
    /// When sensor is mounted with chip facing up:
    /// - NED X (roll rate) = BNO Y (forward rotation)
    /// - NED Y (pitch rate) = BNO X (right rotation)
    /// - NED Z (yaw rate) = -BNO Z (down rotation, inverted)
    pub fn to_angular_velocity_ned(&self) -> Vector3<f32> {
        let sensor = self.to_angular_velocity();
        // Transform from Android frame to NED frame
        // This assumes standard mounting (sensor chip facing up)
        Vector3::new(sensor.y, sensor.x, -sensor.z)
    }

    /// Get status accuracy level (0-3)
    ///
    /// - 0: Unreliable
    /// - 1: Low accuracy
    /// - 2: Medium accuracy
    /// - 3: High accuracy
    pub fn accuracy_status(&self) -> u8 {
        self.status & 0x03
    }
}

/// BNO086 Product ID Response
///
/// Parsed from Report ID 0xF8. Contains device identification.
#[derive(Debug, Clone, Copy, Default)]
pub struct ProductIdResponse {
    /// Reset cause
    pub reset_cause: u8,
    /// Software version major
    pub sw_version_major: u8,
    /// Software version minor
    pub sw_version_minor: u8,
    /// Software part number
    pub sw_part_number: u32,
    /// Software build number
    pub sw_build_number: u32,
    /// Software version patch
    pub sw_version_patch: u16,
}

impl ProductIdResponse {
    /// Minimum payload size for a valid Product ID response
    pub const MIN_PAYLOAD_SIZE: usize = 16;

    /// Parse from SHTP payload bytes
    pub fn parse(payload: &[u8]) -> Option<Self> {
        if payload.len() < Self::MIN_PAYLOAD_SIZE {
            return None;
        }

        // Verify report ID
        if payload[0] != ReportId::ProductIdResponse as u8 {
            return None;
        }

        Some(Self {
            reset_cause: payload[1],
            sw_version_major: payload[2],
            sw_version_minor: payload[3],
            sw_part_number: u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            sw_build_number: u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            sw_version_patch: u16::from_le_bytes([payload[12], payload[13]]),
        })
    }
}

/// SHTP Control Command IDs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlCommand {
    /// Set Feature Command - Enable/configure sensor reports
    SetFeature = 0xFD,
    /// Get Feature Request
    GetFeature = 0xFE,
    /// Product ID Request
    ProductIdRequest = 0xF9,
}

/// Build a "Set Feature Command" payload for a specific sensor
///
/// This command enables a sensor report at the specified interval.
///
/// # Arguments
///
/// * `report_id` - The sensor report ID (e.g., 0x05 for Rotation Vector)
/// * `report_interval_us` - Report interval in microseconds (e.g., 10000 for 100Hz)
///
/// # Returns
///
/// 17-byte payload for the Set Feature Command
pub fn build_set_feature_command_for(report_id: u8, report_interval_us: u32) -> [u8; 17] {
    let interval_bytes = report_interval_us.to_le_bytes();

    [
        ControlCommand::SetFeature as u8, // Command ID (0xFD)
        report_id,                        // Feature Report ID
        0x00,                             // Feature flags
        0x00,                             // Change sensitivity (LSB)
        0x00,                             // Change sensitivity (MSB)
        interval_bytes[0],                // Report interval (LSB)
        interval_bytes[1],
        interval_bytes[2],
        interval_bytes[3], // Report interval (MSB)
        0x00,              // Batch interval (LSB)
        0x00,
        0x00,
        0x00, // Batch interval (MSB)
        0x00, // Sensor-specific config (LSB)
        0x00,
        0x00,
        0x00, // Sensor-specific config (MSB)
    ]
}

/// Build a "Set Feature Command" payload for Rotation Vector
///
/// This command enables the Rotation Vector report at the specified interval.
pub fn build_set_feature_command(report_interval_us: u32) -> [u8; 17] {
    build_set_feature_command_for(ReportId::RotationVector as u8, report_interval_us)
}

/// Build a Product ID Request payload
pub fn build_product_id_request() -> [u8; 2] {
    [ControlCommand::ProductIdRequest as u8, 0x00]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_report_id_from_u8() {
        assert_eq!(ReportId::from_u8(0x05), Some(ReportId::RotationVector));
        assert_eq!(ReportId::from_u8(0x08), Some(ReportId::GameRotationVector));
        assert_eq!(ReportId::from_u8(0xF8), Some(ReportId::ProductIdResponse));
        assert_eq!(ReportId::from_u8(0xFF), None);
    }

    #[test]
    fn test_rotation_vector_parse() {
        // Simulated Rotation Vector report payload
        // Report ID=0x05, seq=1, status=3 (high accuracy)
        // Q values representing identity quaternion (w=1, x=y=z=0)
        // Q14 format: 1.0 = 16384 = 0x4000
        // Structure (CEVA offset 4 base):
        //   Byte 0: Report ID, 1: Seq, 2: Status, 3: Delay
        //   Byte 4-5: Q_i, 6-7: Q_j, 8-9: Q_k, 10-11: Q_real, 12-13: Accuracy
        let payload: [u8; 14] = [
            0x05, // Byte 0: Report ID
            0x01, // Byte 1: Sequence
            0x03, // Byte 2: Status (high accuracy)
            0x00, // Byte 3: Delay
            0x00, 0x00, // Byte 4-5: Q_i = 0 (LE)
            0x00, 0x00, // Byte 6-7: Q_j = 0 (LE)
            0x00, 0x00, // Byte 8-9: Q_k = 0 (LE)
            0x00, 0x40, // Byte 10-11: Q_real = 16384 (LE: 0x4000)
            0x00, 0x00, // Byte 12-13: Accuracy = 0
        ];

        let report = RotationVectorReport::parse(&payload).unwrap();

        assert_eq!(report.sequence, 1);
        assert_eq!(report.status, 3);
        assert_eq!(report.q_i, 0);
        assert_eq!(report.q_j, 0);
        assert_eq!(report.q_k, 0);
        assert_eq!(report.q_real, 16384);
        assert_eq!(report.accuracy_status(), 3);
    }

    #[test]
    fn test_rotation_vector_to_quaternion_identity() {
        let report = RotationVectorReport {
            sequence: 0,
            status: 3,
            q_i: 0,
            q_j: 0,
            q_k: 0,
            q_real: 16384, // 1.0 in Q14
            accuracy: 0,
        };

        let q = report.to_quaternion();

        // Identity quaternion: w=1, x=y=z=0
        assert!((q.w - 1.0).abs() < 0.001);
        assert!(q.i.abs() < 0.001);
        assert!(q.j.abs() < 0.001);
        assert!(q.k.abs() < 0.001);
    }

    #[test]
    fn test_rotation_vector_to_quaternion_90deg_z() {
        // 90 degree rotation around Z axis
        // q = cos(45°) + sin(45°)*k = 0.707 + 0.707*k
        // In Q14: 0.707 * 16384 ≈ 11585
        let report = RotationVectorReport {
            sequence: 0,
            status: 3,
            q_i: 0,
            q_j: 0,
            q_k: 11585,    // ~0.707 in Q14
            q_real: 11585, // ~0.707 in Q14
            accuracy: 0,
        };

        let q = report.to_quaternion();

        // Verify approximately 90 degree rotation
        assert!((q.w - 0.707).abs() < 0.01);
        assert!(q.i.abs() < 0.01);
        assert!(q.j.abs() < 0.01);
        assert!((q.k - 0.707).abs() < 0.01);
    }

    #[test]
    fn test_rotation_vector_accuracy_conversion() {
        let report = RotationVectorReport {
            sequence: 0,
            status: 0,
            q_i: 0,
            q_j: 0,
            q_k: 0,
            q_real: 16384,
            accuracy: 4096, // 1.0 radian in Q12
        };

        let accuracy = report.accuracy_radians();
        assert!((accuracy - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_rotation_vector_parse_too_small() {
        let payload: [u8; 5] = [0x05, 0x01, 0x03, 0x00, 0x00];
        assert!(RotationVectorReport::parse(&payload).is_none());
    }

    #[test]
    fn test_rotation_vector_parse_wrong_report_id() {
        // 14 bytes with wrong report ID (0x08 instead of 0x05)
        let payload: [u8; 14] = [
            0x08, // Wrong Report ID (Game Rotation Vector)
            0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
        ];
        assert!(RotationVectorReport::parse(&payload).is_none());
    }

    #[test]
    fn test_product_id_response_parse() {
        let payload: [u8; 16] = [
            0xF8, // Report ID
            0x01, // Reset cause
            0x03, // SW version major
            0x02, // SW version minor
            0x01, 0x02, 0x03, 0x04, // SW part number
            0x05, 0x06, 0x07, 0x08, // SW build number
            0x09, 0x0A, // SW version patch
            0x00, 0x00, // Padding
        ];

        let response = ProductIdResponse::parse(&payload).unwrap();

        assert_eq!(response.reset_cause, 0x01);
        assert_eq!(response.sw_version_major, 0x03);
        assert_eq!(response.sw_version_minor, 0x02);
        assert_eq!(response.sw_part_number, 0x04030201);
        assert_eq!(response.sw_build_number, 0x08070605);
        assert_eq!(response.sw_version_patch, 0x0A09);
    }

    #[test]
    fn test_build_set_feature_command() {
        // 100Hz = 10000us interval
        let cmd = build_set_feature_command(10000);

        assert_eq!(cmd[0], ControlCommand::SetFeature as u8);
        assert_eq!(cmd[1], ReportId::RotationVector as u8);

        // Verify interval bytes (10000 = 0x2710)
        let interval = u32::from_le_bytes([cmd[5], cmd[6], cmd[7], cmd[8]]);
        assert_eq!(interval, 10000);
    }

    #[test]
    fn test_build_product_id_request() {
        let cmd = build_product_id_request();
        assert_eq!(cmd[0], ControlCommand::ProductIdRequest as u8);
        assert_eq!(cmd[1], 0x00);
    }

    #[test]
    fn test_q14_scale_factor() {
        // Q14 format: value = raw * (1/16384)
        // Raw 16384 should equal 1.0
        assert!((16384.0 * Q14_SCALE - 1.0).abs() < 0.0001);
        // Raw 8192 should equal 0.5
        assert!((8192.0 * Q14_SCALE - 0.5).abs() < 0.0001);
    }

    #[test]
    fn test_q12_scale_factor() {
        // Q12 format: value = raw * (1/4096)
        // Raw 4096 should equal 1.0
        assert!((4096.0 * Q12_SCALE - 1.0).abs() < 0.0001);
        // Raw 2048 should equal 0.5
        assert!((2048.0 * Q12_SCALE - 0.5).abs() < 0.0001);
    }

    #[test]
    fn test_q9_scale_factor() {
        // Q9 format: value = raw * (1/512)
        // Raw 512 should equal 1.0 rad/s
        assert!((512.0 * Q9_SCALE - 1.0).abs() < 0.0001);
        // Raw 256 should equal 0.5 rad/s
        assert!((256.0 * Q9_SCALE - 0.5).abs() < 0.0001);
    }

    #[test]
    fn test_gyroscope_report_parse() {
        // Simulated Gyroscope Calibrated report payload
        // Report ID=0x02, seq=5, status=3 (high accuracy)
        // Gyro values: X=512 (1.0 rad/s), Y=256 (0.5 rad/s), Z=-256 (-0.5 rad/s)
        let payload: [u8; 10] = [
            0x02, // Byte 0: Report ID
            0x05, // Byte 1: Sequence
            0x03, // Byte 2: Status (high accuracy)
            0x00, // Byte 3: Delay
            0x00, 0x02, // Byte 4-5: Gyro X = 512 (LE: 0x0200)
            0x00, 0x01, // Byte 6-7: Gyro Y = 256 (LE: 0x0100)
            0x00, 0xFF, // Byte 8-9: Gyro Z = -256 (LE: 0xFF00)
        ];

        let report = GyroscopeReport::parse(&payload).unwrap();

        assert_eq!(report.sequence, 5);
        assert_eq!(report.status, 3);
        assert_eq!(report.gyro_x, 512);
        assert_eq!(report.gyro_y, 256);
        assert_eq!(report.gyro_z, -256);
        assert_eq!(report.accuracy_status(), 3);
    }

    #[test]
    fn test_gyroscope_report_to_angular_velocity() {
        let report = GyroscopeReport {
            sequence: 0,
            status: 3,
            gyro_x: 512,  // 1.0 rad/s
            gyro_y: 256,  // 0.5 rad/s
            gyro_z: -512, // -1.0 rad/s
        };

        let v = report.to_angular_velocity();

        assert!((v.x - 1.0).abs() < 0.001);
        assert!((v.y - 0.5).abs() < 0.001);
        assert!((v.z + 1.0).abs() < 0.001);
    }

    #[test]
    fn test_gyroscope_report_to_angular_velocity_ned() {
        // Test NED frame conversion
        // Sensor frame: X=1.0, Y=0.5, Z=-1.0
        // NED frame: X=Y_sensor=0.5, Y=X_sensor=1.0, Z=-Z_sensor=1.0
        let report = GyroscopeReport {
            sequence: 0,
            status: 3,
            gyro_x: 512,  // 1.0 rad/s
            gyro_y: 256,  // 0.5 rad/s
            gyro_z: -512, // -1.0 rad/s
        };

        let v_ned = report.to_angular_velocity_ned();

        assert!((v_ned.x - 0.5).abs() < 0.001); // NED X = sensor Y
        assert!((v_ned.y - 1.0).abs() < 0.001); // NED Y = sensor X
        assert!((v_ned.z - 1.0).abs() < 0.001); // NED Z = -sensor Z
    }

    #[test]
    fn test_gyroscope_report_parse_too_small() {
        let payload: [u8; 5] = [0x02, 0x01, 0x03, 0x00, 0x00];
        assert!(GyroscopeReport::parse(&payload).is_none());
    }

    #[test]
    fn test_gyroscope_report_parse_wrong_report_id() {
        // 10 bytes with wrong report ID (0x05 instead of 0x02)
        let payload: [u8; 10] = [
            0x05, // Wrong Report ID (Rotation Vector)
            0x01, 0x03, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00, 0xFF,
        ];
        assert!(GyroscopeReport::parse(&payload).is_none());
    }
}
