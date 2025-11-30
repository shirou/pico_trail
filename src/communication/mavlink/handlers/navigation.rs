//! Navigation Protocol Handler
//!
//! Handles SET_POSITION_TARGET_GLOBAL_INT messages for guided navigation.
//!
//! # Message Flow
//!
//! 1. GCS sends SET_POSITION_TARGET_GLOBAL_INT with target position
//! 2. Handler extracts latitude/longitude from message
//! 3. Handler updates global NAV_TARGET state
//! 4. navigation_task reads NAV_TARGET and computes steering/throttle
//!
//! # Coordinate Format
//!
//! MAVLink uses lat_int/lon_int as degrees * 1e7 (integer format).
//! This handler converts to f32 degrees for internal use.

use crate::subsystems::navigation::PositionTarget;
use mavlink::common::SET_POSITION_TARGET_GLOBAL_INT_DATA;

/// Navigation handler for SET_POSITION_TARGET_GLOBAL_INT messages
///
/// Stateless handler that converts MAVLink position targets to internal format.
#[derive(Default)]
pub struct NavigationHandler {}

impl NavigationHandler {
    /// Create a new navigation handler
    pub fn new() -> Self {
        Self::default()
    }

    /// Handle SET_POSITION_TARGET_GLOBAL_INT message
    ///
    /// Extracts position from MAVLink message and updates global NAV_TARGET.
    ///
    /// # Arguments
    ///
    /// * `data` - MAVLink SET_POSITION_TARGET_GLOBAL_INT message data
    ///
    /// # Returns
    ///
    /// `PositionTarget` extracted from the message, or `None` if invalid
    ///
    /// # Coordinate Conversion
    ///
    /// MAVLink lat_int/lon_int are in degE7 (degrees * 10^7).
    /// Example: 356762000 = 35.6762 degrees
    pub fn extract_target(
        &self,
        data: &SET_POSITION_TARGET_GLOBAL_INT_DATA,
    ) -> Option<PositionTarget> {
        // Convert from degE7 to degrees
        let latitude = data.lat_int as f32 / 1e7;
        let longitude = data.lon_int as f32 / 1e7;

        // Validate coordinates
        if !(-90.0..=90.0).contains(&latitude) || !(-180.0..=180.0).contains(&longitude) {
            crate::log_warn!(
                "Invalid position target: lat={}, lon={}",
                latitude,
                longitude
            );
            return None;
        }

        // Extract altitude if present (alt field is in mm, convert to m)
        let altitude = if data.alt.is_finite() && data.alt != 0.0 {
            Some(data.alt)
        } else {
            None
        };

        Some(PositionTarget {
            latitude,
            longitude,
            altitude,
        })
    }

    /// Update global NAV_TARGET with new position (async)
    ///
    /// # Arguments
    ///
    /// * `data` - MAVLink SET_POSITION_TARGET_GLOBAL_INT message data
    ///
    /// # Returns
    ///
    /// `true` if target was successfully updated, `false` if invalid
    #[cfg(feature = "pico2_w")]
    pub async fn handle_set_position_target(
        &self,
        data: &SET_POSITION_TARGET_GLOBAL_INT_DATA,
    ) -> bool {
        use crate::subsystems::navigation::NAV_TARGET;

        if let Some(target) = self.extract_target(data) {
            crate::log_info!(
                "Navigation target set: lat={}, lon={}",
                target.latitude,
                target.longitude
            );

            // Update global NAV_TARGET
            let mut nav_target = NAV_TARGET.lock().await;
            *nav_target = Some(target);
            true
        } else {
            false
        }
    }

    /// Synchronous version for non-async contexts (host tests)
    #[cfg(not(feature = "pico2_w"))]
    pub fn handle_set_position_target(&self, data: &SET_POSITION_TARGET_GLOBAL_INT_DATA) -> bool {
        self.extract_target(data).is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_data(lat_e7: i32, lon_e7: i32, alt: f32) -> SET_POSITION_TARGET_GLOBAL_INT_DATA {
        #[allow(deprecated)]
        SET_POSITION_TARGET_GLOBAL_INT_DATA {
            time_boot_ms: 0,
            target_system: 1,
            target_component: 1,
            coordinate_frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            type_mask: mavlink::common::PositionTargetTypemask::empty(),
            lat_int: lat_e7,
            lon_int: lon_e7,
            alt,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.0,
            yaw_rate: 0.0,
        }
    }

    #[test]
    fn test_extract_target_valid() {
        let handler = NavigationHandler::new();

        // Tokyo: 35.6762, 139.6503
        let data = create_test_data(356762000, 1396503000, 0.0);
        let target = handler.extract_target(&data);

        assert!(target.is_some());
        let target = target.unwrap();
        assert!((target.latitude - 35.6762).abs() < 0.0001);
        assert!((target.longitude - 139.6503).abs() < 0.0001);
        assert!(target.altitude.is_none());
    }

    #[test]
    fn test_extract_target_with_altitude() {
        let handler = NavigationHandler::new();

        let data = create_test_data(356762000, 1396503000, 100.0);
        let target = handler.extract_target(&data);

        assert!(target.is_some());
        let target = target.unwrap();
        assert!((target.altitude.unwrap() - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_extract_target_negative_coords() {
        let handler = NavigationHandler::new();

        // New York: 40.7128, -74.0060
        let data = create_test_data(407128000, -740060000, 0.0);
        let target = handler.extract_target(&data);

        assert!(target.is_some());
        let target = target.unwrap();
        assert!((target.latitude - 40.7128).abs() < 0.0001);
        assert!((target.longitude - (-74.0060)).abs() < 0.0001);
    }

    #[test]
    fn test_extract_target_invalid_latitude() {
        let handler = NavigationHandler::new();

        // Invalid latitude (> 90)
        let data = create_test_data(910000000, 0, 0.0);
        let target = handler.extract_target(&data);

        assert!(target.is_none());
    }

    #[test]
    fn test_extract_target_invalid_longitude() {
        let handler = NavigationHandler::new();

        // Invalid longitude (> 180)
        let data = create_test_data(0, 1810000000, 0.0);
        let target = handler.extract_target(&data);

        assert!(target.is_none());
    }
}
