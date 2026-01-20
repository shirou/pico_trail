//! Navigation calculation utilities
//!
//! Pure functions for navigation calculations used by autonomous modes.

use libm::{atan2f, cosf, sinf, sqrtf};

/// Calculate distance and bearing between two GPS positions using Haversine formula
///
/// # Arguments
///
/// * `lat1`, `lon1` - Start position in degrees
/// * `lat2`, `lon2` - End position in degrees
///
/// # Returns
///
/// Tuple of (distance in meters, bearing in degrees 0-360)
pub fn haversine_distance_bearing(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> (f32, f32) {
    const EARTH_RADIUS_M: f32 = 6_371_000.0;
    const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
    const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

    let lat1_rad = lat1 * DEG_TO_RAD;
    let lat2_rad = lat2 * DEG_TO_RAD;
    let delta_lat = (lat2 - lat1) * DEG_TO_RAD;
    let delta_lon = (lon2 - lon1) * DEG_TO_RAD;

    // Haversine formula for distance
    let sin_dlat = sinf(delta_lat / 2.0);
    let sin_dlon = sinf(delta_lon / 2.0);
    let a = sin_dlat * sin_dlat + cosf(lat1_rad) * cosf(lat2_rad) * sin_dlon * sin_dlon;
    let c = 2.0 * atan2f(sqrtf(a), sqrtf(1.0 - a));
    let distance = EARTH_RADIUS_M * c;

    // Forward azimuth (bearing)
    let y = sinf(delta_lon) * cosf(lat2_rad);
    let x = cosf(lat1_rad) * sinf(lat2_rad) - sinf(lat1_rad) * cosf(lat2_rad) * cosf(delta_lon);
    let bearing = atan2f(y, x) * RAD_TO_DEG;
    let bearing = (bearing + 360.0) % 360.0;

    (distance, bearing)
}

/// Normalize angle to -180 to +180 range
///
/// # Arguments
///
/// * `angle` - Input angle in degrees
///
/// # Returns
///
/// Normalized angle in degrees (-180 to +180)
pub fn normalize_angle(angle: f32) -> f32 {
    let mut a = angle % 360.0;
    if a > 180.0 {
        a -= 360.0;
    } else if a < -180.0 {
        a += 360.0;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine_distance_north() {
        // Test known distance: ~111km per degree of latitude
        let (distance, _bearing) = haversine_distance_bearing(35.0, 139.0, 36.0, 139.0);
        // Should be approximately 111km
        assert!((distance - 111_000.0).abs() < 1000.0);
    }

    #[test]
    fn test_haversine_bearing_north() {
        let (_distance, bearing) = haversine_distance_bearing(35.0, 139.0, 36.0, 139.0);
        // Target is north, bearing should be ~0
        assert!(bearing.abs() < 1.0 || (bearing - 360.0).abs() < 1.0);
    }

    #[test]
    fn test_haversine_bearing_east() {
        let (_distance, bearing) = haversine_distance_bearing(35.0, 139.0, 35.0, 140.0);
        // Target is east, bearing should be ~90
        assert!((bearing - 90.0).abs() < 1.0);
    }

    #[test]
    fn test_haversine_bearing_south() {
        let (_distance, bearing) = haversine_distance_bearing(36.0, 139.0, 35.0, 139.0);
        // Target is south, bearing should be ~180
        assert!((bearing - 180.0).abs() < 1.0);
    }

    #[test]
    fn test_haversine_bearing_west() {
        let (_distance, bearing) = haversine_distance_bearing(35.0, 140.0, 35.0, 139.0);
        // Target is west, bearing should be ~270
        assert!((bearing - 270.0).abs() < 1.0);
    }

    #[test]
    fn test_normalize_angle_zero() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_normalize_angle_180() {
        assert!((normalize_angle(180.0) - 180.0).abs() < 0.001);
    }

    #[test]
    fn test_normalize_angle_negative_180() {
        assert!((normalize_angle(-180.0) - (-180.0)).abs() < 0.001);
    }

    #[test]
    fn test_normalize_angle_270() {
        assert!((normalize_angle(270.0) - (-90.0)).abs() < 0.001);
    }

    #[test]
    fn test_normalize_angle_negative_270() {
        assert!((normalize_angle(-270.0) - 90.0).abs() < 0.001);
    }

    #[test]
    fn test_normalize_angle_450() {
        assert!((normalize_angle(450.0) - 90.0).abs() < 0.001);
    }
}
