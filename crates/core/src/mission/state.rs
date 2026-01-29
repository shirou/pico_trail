//! Mission State Types
//!
//! Pure data types for mission execution state tracking.
//!
//! Global state management and Embassy-specific wrappers are
//! provided by the firmware crate.

use crate::navigation::PositionTarget;

use super::Waypoint;

/// Mission execution state
///
/// Tracks the current state of mission execution for both GUIDED and AUTO modes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MissionState {
    /// No mission active, navigation idle
    #[default]
    Idle,
    /// Mission running, navigating to waypoints
    Running,
    /// Mission completed, all waypoints reached
    Completed,
}

// ============================================================================
// Waypoint to PositionTarget Conversion
// ============================================================================

impl From<&Waypoint> for PositionTarget {
    fn from(wp: &Waypoint) -> Self {
        Self {
            latitude: (wp.x as f64 / 1e7) as f32,
            longitude: (wp.y as f64 / 1e7) as f32,
            altitude: Some(wp.z),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mission_state_default() {
        let state = MissionState::default();
        assert_eq!(state, MissionState::Idle);
    }

    #[test]
    fn test_mission_state_equality() {
        assert_eq!(MissionState::Idle, MissionState::Idle);
        assert_eq!(MissionState::Running, MissionState::Running);
        assert_eq!(MissionState::Completed, MissionState::Completed);
        assert_ne!(MissionState::Idle, MissionState::Running);
    }

    #[test]
    fn test_mission_state_clone() {
        let state = MissionState::Running;
        let cloned = state;
        assert_eq!(state, cloned);
    }

    #[test]
    fn test_waypoint_to_position_target_conversion() {
        let wp = Waypoint {
            seq: 0,
            frame: 3,
            command: 16,
            current: 0,
            autocontinue: 1,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 357000000,  // 35.7 degrees
            y: 1396000000, // 139.6 degrees
            z: 100.0,
        };

        let target = PositionTarget::from(&wp);
        assert!((target.latitude - 35.7).abs() < 0.0001);
        assert!((target.longitude - 139.6).abs() < 0.0001);
        assert_eq!(target.altitude, Some(100.0));
    }

    #[test]
    fn test_waypoint_to_position_target_negative_coords() {
        let wp = Waypoint {
            seq: 0,
            frame: 3,
            command: 16,
            current: 0,
            autocontinue: 1,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: -337000000,  // -33.7 degrees (south)
            y: -1512000000, // -151.2 degrees (west)
            z: 50.0,
        };

        let target = PositionTarget::from(&wp);
        assert!((target.latitude - (-33.7)).abs() < 0.0001);
        assert!((target.longitude - (-151.2)).abs() < 0.0001);
        assert_eq!(target.altitude, Some(50.0));
    }
}
