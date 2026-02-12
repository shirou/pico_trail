//! Navigation Protocol Handler
//!
//! Handles SET_POSITION_TARGET_GLOBAL_INT messages for guided navigation.

use crate::navigation::PositionTarget;
use mavlink::common::SET_POSITION_TARGET_GLOBAL_INT_DATA;

/// Navigation handler for SET_POSITION_TARGET_GLOBAL_INT messages
#[derive(Default)]
pub struct NavigationHandler {}

impl NavigationHandler {
    /// Create a new navigation handler
    pub fn new() -> Self {
        Self::default()
    }

    /// Extract position target from MAVLink message
    pub fn extract_target(
        &self,
        data: &SET_POSITION_TARGET_GLOBAL_INT_DATA,
    ) -> Option<PositionTarget> {
        let latitude = data.lat_int as f32 / 1e7;
        let longitude = data.lon_int as f32 / 1e7;

        if !(-90.0..=90.0).contains(&latitude) || !(-180.0..=180.0).contains(&longitude) {
            crate::log_warn!(
                "Invalid position target: lat={}, lon={}",
                latitude,
                longitude
            );
            return None;
        }

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

    /// Update MissionStorage with new position target (async)
    pub async fn handle_set_position_target(
        &self,
        data: &SET_POSITION_TARGET_GLOBAL_INT_DATA,
    ) -> bool {
        use crate::autopilot::state::{FlightMode, SYSTEM_STATE};
        use crate::mission::{set_mission_state, set_single_waypoint, MissionState, Waypoint};

        if let Some(target) = self.extract_target(data) {
            crate::log_info!(
                "Navigation target set: lat={}, lon={}",
                target.latitude,
                target.longitude
            );

            let waypoint = Waypoint {
                seq: 0,
                frame: 0,
                command: 16,
                current: 1,
                autocontinue: 0,
                param1: 0.0,
                param2: 2.0,
                param3: 0.0,
                param4: 0.0,
                x: data.lat_int,
                y: data.lon_int,
                z: target.altitude.unwrap_or(0.0),
            };

            set_single_waypoint(waypoint);

            let is_guided = critical_section::with(|cs| {
                let state = SYSTEM_STATE.borrow_ref(cs);
                state.mode == FlightMode::Guided
            });

            if is_guided {
                set_mission_state(MissionState::Running);
                crate::log_info!("GUIDED mode: Starting navigation to target");
            }

            true
        } else {
            false
        }
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
        assert!((target.unwrap().altitude.unwrap() - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_extract_target_negative_coords() {
        let handler = NavigationHandler::new();
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
        let data = create_test_data(910000000, 0, 0.0);
        assert!(handler.extract_target(&data).is_none());
    }

    #[test]
    fn test_extract_target_invalid_longitude() {
        let handler = NavigationHandler::new();
        let data = create_test_data(0, 1810000000, 0.0);
        assert!(handler.extract_target(&data).is_none());
    }
}
