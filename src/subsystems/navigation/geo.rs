//! Geographic calculations for navigation
//!
//! This module provides functions for calculating:
//! - Bearing between two points (initial bearing formula)
//! - Distance between two points (Haversine formula)
//! - Angle wrapping utilities

use libm::{asinf, atan2f, cosf, sinf, sqrtf};

/// Earth radius in meters
const EARTH_RADIUS_M: f32 = 6_371_000.0;

/// Degrees to radians conversion factor
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

/// Radians to degrees conversion factor
const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

/// Calculate the initial bearing from point 1 to point 2
///
/// Uses the initial bearing formula:
/// θ = atan2(sin(Δlon) × cos(lat2),
///           cos(lat1) × sin(lat2) - sin(lat1) × cos(lat2) × cos(Δlon))
///
/// # Arguments
/// * `lat1` - Latitude of point 1 in degrees
/// * `lon1` - Longitude of point 1 in degrees
/// * `lat2` - Latitude of point 2 in degrees
/// * `lon2` - Longitude of point 2 in degrees
///
/// # Returns
/// Bearing in degrees (0-360, where 0 = North, 90 = East)
pub fn calculate_bearing(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
    let lat1_rad = lat1 * DEG_TO_RAD;
    let lat2_rad = lat2 * DEG_TO_RAD;
    let delta_lon_rad = (lon2 - lon1) * DEG_TO_RAD;

    let x = sinf(delta_lon_rad) * cosf(lat2_rad);
    let y = cosf(lat1_rad) * sinf(lat2_rad) - sinf(lat1_rad) * cosf(lat2_rad) * cosf(delta_lon_rad);

    let bearing_rad = atan2f(x, y);
    let bearing_deg = bearing_rad * RAD_TO_DEG;

    // Normalize to 0-360 range
    wrap_360(bearing_deg)
}

/// Calculate the distance between two points using the Haversine formula
///
/// Formula:
/// a = sin²(Δlat/2) + cos(lat1) × cos(lat2) × sin²(Δlon/2)
/// c = 2 × atan2(√a, √(1-a))
/// d = R × c
///
/// # Arguments
/// * `lat1` - Latitude of point 1 in degrees
/// * `lon1` - Longitude of point 1 in degrees
/// * `lat2` - Latitude of point 2 in degrees
/// * `lon2` - Longitude of point 2 in degrees
///
/// # Returns
/// Distance in meters
pub fn calculate_distance(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
    let lat1_rad = lat1 * DEG_TO_RAD;
    let lat2_rad = lat2 * DEG_TO_RAD;
    let delta_lat_rad = (lat2 - lat1) * DEG_TO_RAD;
    let delta_lon_rad = (lon2 - lon1) * DEG_TO_RAD;

    let sin_dlat_2 = sinf(delta_lat_rad / 2.0);
    let sin_dlon_2 = sinf(delta_lon_rad / 2.0);

    let a = sin_dlat_2 * sin_dlat_2 + cosf(lat1_rad) * cosf(lat2_rad) * sin_dlon_2 * sin_dlon_2;

    let c = 2.0 * atan2f(sqrtf(a), sqrtf(1.0 - a));

    EARTH_RADIUS_M * c
}

/// Wrap an angle to the range [-180, +180] degrees
///
/// # Arguments
/// * `angle` - Angle in degrees
///
/// # Returns
/// Angle wrapped to [-180, +180] range
pub fn wrap_180(angle: f32) -> f32 {
    let mut result = angle;

    // Handle large angles efficiently
    while result > 180.0 {
        result -= 360.0;
    }
    while result < -180.0 {
        result += 360.0;
    }

    result
}

/// Wrap an angle to the range [0, 360) degrees
///
/// # Arguments
/// * `angle` - Angle in degrees
///
/// # Returns
/// Angle wrapped to [0, 360) range
pub fn wrap_360(angle: f32) -> f32 {
    let mut result = angle;

    // Handle large angles efficiently
    while result >= 360.0 {
        result -= 360.0;
    }
    while result < 0.0 {
        result += 360.0;
    }

    result
}

/// Calculate destination point given start point, distance and bearing
///
/// Uses the spherical law of cosines to calculate the destination point.
///
/// Formula:
/// φ₂ = asin(sin(φ₁)·cos(d/R) + cos(φ₁)·sin(d/R)·cos(θ))
/// λ₂ = λ₁ + atan2(sin(θ)·sin(d/R)·cos(φ₁), cos(d/R) − sin(φ₁)·sin(φ₂))
///
/// # Arguments
/// * `lat` - Latitude of start point in degrees
/// * `lon` - Longitude of start point in degrees
/// * `distance` - Distance to destination in meters
/// * `bearing` - Bearing to destination in degrees (0-360, where 0 = North)
///
/// # Returns
/// Tuple of (latitude, longitude) in degrees
pub fn offset_position(lat: f32, lon: f32, distance: f32, bearing: f32) -> (f32, f32) {
    // Handle zero distance case
    if distance <= 0.0 {
        return (lat, lon);
    }

    let lat_rad = lat * DEG_TO_RAD;
    let lon_rad = lon * DEG_TO_RAD;
    let bearing_rad = bearing * DEG_TO_RAD;
    let angular_distance = distance / EARTH_RADIUS_M;

    // Calculate destination latitude
    let sin_lat1 = sinf(lat_rad);
    let cos_lat1 = cosf(lat_rad);
    let cos_d = cosf(angular_distance);
    let sin_d = sinf(angular_distance);

    let sin_lat2 = sin_lat1 * cos_d + cos_lat1 * sin_d * cosf(bearing_rad);
    // Clamp to [-1, 1] to handle floating point errors
    let sin_lat2_clamped = sin_lat2.clamp(-1.0, 1.0);
    let lat2_rad = asinf(sin_lat2_clamped);

    // Calculate destination longitude
    let y = sinf(bearing_rad) * sin_d * cos_lat1;
    let x = cos_d - sin_lat1 * sinf(lat2_rad);
    let lon2_rad = lon_rad + atan2f(y, x);

    // Convert back to degrees and normalize longitude
    let lat2 = lat2_rad * RAD_TO_DEG;
    let mut lon2 = lon2_rad * RAD_TO_DEG;

    // Normalize longitude to [-180, +180]
    while lon2 > 180.0 {
        lon2 -= 360.0;
    }
    while lon2 < -180.0 {
        lon2 += 360.0;
    }

    (lat2, lon2)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.5; // 0.5 degree tolerance for bearing
    const DIST_EPSILON: f32 = 1000.0; // 1km tolerance for long distances

    // ========== Bearing Tests ==========

    #[test]
    fn test_bearing_north() {
        // From (0, 0) to (1, 0) should be 0° (north)
        let bearing = calculate_bearing(0.0, 0.0, 1.0, 0.0);
        assert!(
            (bearing - 0.0).abs() < EPSILON,
            "Expected ~0°, got {}°",
            bearing
        );
    }

    #[test]
    fn test_bearing_south() {
        // From (0, 0) to (-1, 0) should be 180° (south)
        let bearing = calculate_bearing(0.0, 0.0, -1.0, 0.0);
        assert!(
            (bearing - 180.0).abs() < EPSILON,
            "Expected ~180°, got {}°",
            bearing
        );
    }

    #[test]
    fn test_bearing_east() {
        // From (0, 0) to (0, 1) should be 90° (east)
        let bearing = calculate_bearing(0.0, 0.0, 0.0, 1.0);
        assert!(
            (bearing - 90.0).abs() < EPSILON,
            "Expected ~90°, got {}°",
            bearing
        );
    }

    #[test]
    fn test_bearing_west() {
        // From (0, 0) to (0, -1) should be 270° (west)
        let bearing = calculate_bearing(0.0, 0.0, 0.0, -1.0);
        assert!(
            (bearing - 270.0).abs() < EPSILON,
            "Expected ~270°, got {}°",
            bearing
        );
    }

    #[test]
    fn test_bearing_northeast() {
        // From (0, 0) to (1, 1) should be ~45° (northeast)
        let bearing = calculate_bearing(0.0, 0.0, 1.0, 1.0);
        assert!(
            (bearing - 45.0).abs() < 1.0,
            "Expected ~45°, got {}°",
            bearing
        );
    }

    #[test]
    fn test_bearing_southwest() {
        // From (0, 0) to (-1, -1) should be ~225° (southwest)
        let bearing = calculate_bearing(0.0, 0.0, -1.0, -1.0);
        assert!(
            (bearing - 225.0).abs() < 1.0,
            "Expected ~225°, got {}°",
            bearing
        );
    }

    // ========== Distance Tests ==========

    #[test]
    fn test_distance_zero() {
        // Same point should have zero distance
        let dist = calculate_distance(35.6762, 139.6503, 35.6762, 139.6503);
        assert!(dist < 1.0, "Expected ~0m, got {}m", dist);
    }

    #[test]
    fn test_distance_tokyo_to_osaka() {
        // Tokyo (35.6762°N, 139.6503°E) to Osaka (34.6937°N, 135.5023°E)
        // Approximately 400km
        let dist = calculate_distance(35.6762, 139.6503, 34.6937, 135.5023);
        let expected = 400_000.0;
        assert!(
            (dist - expected).abs() < 20_000.0,
            "Expected ~400km, got {}km",
            dist / 1000.0
        );
    }

    #[test]
    fn test_distance_short() {
        // About 1 degree latitude = 111km
        let dist = calculate_distance(0.0, 0.0, 1.0, 0.0);
        let expected = 111_000.0;
        assert!(
            (dist - expected).abs() < DIST_EPSILON,
            "Expected ~111km, got {}km",
            dist / 1000.0
        );
    }

    #[test]
    fn test_distance_small() {
        // Very small distance (about 1 meter)
        // 1 meter ≈ 0.000009 degrees of latitude
        let dist = calculate_distance(0.0, 0.0, 0.000009, 0.0);
        assert!(dist < 2.0 && dist > 0.5, "Expected ~1m, got {}m", dist);
    }

    // ========== wrap_180 Tests ==========

    #[test]
    fn test_wrap_180_zero() {
        assert!((wrap_180(0.0) - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_wrap_180_positive_in_range() {
        assert!((wrap_180(90.0) - 90.0).abs() < 0.001);
        assert!((wrap_180(180.0) - 180.0).abs() < 0.001);
    }

    #[test]
    fn test_wrap_180_negative_in_range() {
        assert!((wrap_180(-90.0) - (-90.0)).abs() < 0.001);
        assert!((wrap_180(-180.0) - (-180.0)).abs() < 0.001);
    }

    #[test]
    fn test_wrap_180_positive_overflow() {
        assert!((wrap_180(270.0) - (-90.0)).abs() < 0.001);
        assert!((wrap_180(360.0) - 0.0).abs() < 0.001);
        assert!((wrap_180(450.0) - 90.0).abs() < 0.001);
    }

    #[test]
    fn test_wrap_180_negative_overflow() {
        assert!((wrap_180(-270.0) - 90.0).abs() < 0.001);
        assert!((wrap_180(-360.0) - 0.0).abs() < 0.001);
        assert!((wrap_180(-450.0) - (-90.0)).abs() < 0.001);
    }

    // ========== wrap_360 Tests ==========

    #[test]
    fn test_wrap_360_zero() {
        assert!((wrap_360(0.0) - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_wrap_360_in_range() {
        assert!((wrap_360(90.0) - 90.0).abs() < 0.001);
        assert!((wrap_360(180.0) - 180.0).abs() < 0.001);
        assert!((wrap_360(270.0) - 270.0).abs() < 0.001);
    }

    #[test]
    fn test_wrap_360_positive_overflow() {
        assert!((wrap_360(360.0) - 0.0).abs() < 0.001);
        assert!((wrap_360(450.0) - 90.0).abs() < 0.001);
        assert!((wrap_360(720.0) - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_wrap_360_negative() {
        assert!((wrap_360(-90.0) - 270.0).abs() < 0.001);
        assert!((wrap_360(-180.0) - 180.0).abs() < 0.001);
        assert!((wrap_360(-270.0) - 90.0).abs() < 0.001);
        assert!((wrap_360(-360.0) - 0.0).abs() < 0.001);
    }

    // ========== Pseudo-Property Tests ==========

    #[test]
    fn test_wrap_180_always_in_range() {
        // Test many values to verify wrap_180 always returns [-180, 180]
        for angle in (-720..=720).step_by(15) {
            let result = wrap_180(angle as f32);
            assert!(
                (-180.0..=180.0).contains(&result),
                "wrap_180({}) = {} is out of range [-180, 180]",
                angle,
                result
            );
        }
    }

    #[test]
    fn test_wrap_360_always_in_range() {
        // Test many values to verify wrap_360 always returns [0, 360)
        for angle in (-720..=720).step_by(15) {
            let result = wrap_360(angle as f32);
            assert!(
                (0.0..360.0).contains(&result),
                "wrap_360({}) = {} is out of range [0, 360)",
                angle,
                result
            );
        }
    }

    #[test]
    fn test_bearing_always_in_range() {
        // Test bearing for various lat/lon combinations
        let coords: [(f32, f32); 6] = [
            (0.0, 0.0),
            (45.0, 90.0),
            (-45.0, -90.0),
            (89.0, 0.0),
            (0.0, 179.0),
            (0.0, -179.0),
        ];

        for (lat1, lon1) in coords.iter() {
            for (lat2, lon2) in coords.iter() {
                if (lat1 - lat2).abs() > 0.01 || (lon1 - lon2).abs() > 0.01 {
                    let bearing = calculate_bearing(*lat1, *lon1, *lat2, *lon2);
                    assert!(
                        (0.0..360.0).contains(&bearing),
                        "Bearing from ({}, {}) to ({}, {}) = {} is out of range",
                        lat1,
                        lon1,
                        lat2,
                        lon2,
                        bearing
                    );
                }
            }
        }
    }

    #[test]
    fn test_distance_always_non_negative() {
        // Test distance for various lat/lon combinations
        let coords: [(f32, f32); 5] = [
            (0.0, 0.0),
            (45.0, 90.0),
            (-45.0, -90.0),
            (89.0, 0.0),
            (0.0, 179.0),
        ];

        for (lat1, lon1) in coords.iter() {
            for (lat2, lon2) in coords.iter() {
                let dist = calculate_distance(*lat1, *lon1, *lat2, *lon2);
                assert!(
                    dist >= 0.0,
                    "Distance from ({}, {}) to ({}, {}) = {} is negative",
                    lat1,
                    lon1,
                    lat2,
                    lon2,
                    dist
                );
                // Also check not NaN
                assert!(
                    !dist.is_nan(),
                    "Distance from ({}, {}) to ({}, {}) is NaN",
                    lat1,
                    lon1,
                    lat2,
                    lon2
                );
            }
        }
    }

    // ========== offset_position Tests ==========

    #[test]
    fn test_offset_position_north() {
        // Move 1000m north from equator/prime meridian
        let (lat, lon) = offset_position(0.0, 0.0, 1000.0, 0.0);
        // 1000m north should increase latitude by ~0.009 degrees
        assert!(
            lat > 0.008 && lat < 0.010,
            "Expected lat ~0.009°, got {}°",
            lat
        );
        assert!(lon.abs() < 0.001, "Expected lon ~0°, got {}°", lon);
    }

    #[test]
    fn test_offset_position_south() {
        let (lat, lon) = offset_position(0.0, 0.0, 1000.0, 180.0);
        assert!(
            lat < -0.008 && lat > -0.010,
            "Expected lat ~-0.009°, got {}°",
            lat
        );
        assert!(lon.abs() < 0.001, "Expected lon ~0°, got {}°", lon);
    }

    #[test]
    fn test_offset_position_east() {
        let (lat, lon) = offset_position(0.0, 0.0, 1000.0, 90.0);
        assert!(lat.abs() < 0.001, "Expected lat ~0°, got {}°", lat);
        assert!(
            lon > 0.008 && lon < 0.010,
            "Expected lon ~0.009°, got {}°",
            lon
        );
    }

    #[test]
    fn test_offset_position_west() {
        let (lat, lon) = offset_position(0.0, 0.0, 1000.0, 270.0);
        assert!(lat.abs() < 0.001, "Expected lat ~0°, got {}°", lat);
        assert!(
            lon < -0.008 && lon > -0.010,
            "Expected lon ~-0.009°, got {}°",
            lon
        );
    }

    #[test]
    fn test_offset_position_zero_distance() {
        let (lat, lon) = offset_position(35.6762, 139.6503, 0.0, 45.0);
        assert!(
            (lat - 35.6762).abs() < 0.0001,
            "Expected lat 35.6762°, got {}°",
            lat
        );
        assert!(
            (lon - 139.6503).abs() < 0.0001,
            "Expected lon 139.6503°, got {}°",
            lon
        );
    }

    #[test]
    fn test_offset_position_roundtrip() {
        // Offset then calculate distance should match original
        let start_lat = 35.6762;
        let start_lon = 139.6503;
        let distance = 500.0;
        let bearing = 45.0;

        let (end_lat, end_lon) = offset_position(start_lat, start_lon, distance, bearing);
        let calc_distance = calculate_distance(start_lat, start_lon, end_lat, end_lon);

        assert!(
            (calc_distance - distance).abs() < 1.0,
            "Expected distance {}m, got {}m",
            distance,
            calc_distance
        );
    }

    #[test]
    fn test_offset_position_all_bearings() {
        let start_lat = 0.0;
        let start_lon = 0.0;
        let distance = 1000.0;

        for bearing in (0..360).step_by(30) {
            let (lat, lon) = offset_position(start_lat, start_lon, distance, bearing as f32);
            assert!(
                !lat.is_nan() && !lon.is_nan(),
                "NaN result for bearing {}°",
                bearing
            );
            assert!(
                (-90.0..=90.0).contains(&lat),
                "Latitude out of range for bearing {}°: {}",
                bearing,
                lat
            );
            assert!(
                (-180.0..=180.0).contains(&lon),
                "Longitude out of range for bearing {}°: {}",
                bearing,
                lon
            );
        }
    }
}
