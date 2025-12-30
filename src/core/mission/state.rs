//! Mission State Management
//!
//! Provides mission execution state tracking and global mission storage.
//!
//! # Architecture
//!
//! MissionStorage serves as the single source of truth for waypoint navigation
//! in both GUIDED and AUTO modes. This module provides:
//!
//! - MissionState enum for tracking execution status
//! - Global MISSION_STORAGE accessible from navigation and handlers
//! - Helper functions for synchronized state access
//!
//! # Migration Note (FR-jpmdj)
//!
//! This module uses `EmbassyState` for synchronized access, which provides
//! sync (non-async) operations via critical sections. All functions are now
//! synchronous, eliminating the need for separate `_sync` variants.

use super::Waypoint;
use crate::subsystems::navigation::PositionTarget;

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
// Embassy Implementation (embedded targets)
// ============================================================================

#[cfg(feature = "embassy")]
mod embassy_impl {
    use super::{MissionState, PositionTarget, Waypoint};
    use crate::core::mission::MissionStorage;
    use crate::core::traits::{EmbassyState, SharedState};

    /// Global mission state (protected by EmbassyState)
    ///
    /// Tracks mission execution status (Idle/Running/Completed).
    /// Updated by mode handlers and navigation task.
    /// Uses blocking mutex with critical sections for interrupt-safe access.
    pub static MISSION_STATE: EmbassyState<MissionState> = EmbassyState::new(MissionState::Idle);

    /// Global mission storage (protected by EmbassyState)
    ///
    /// Single source of truth for waypoint navigation.
    /// Updated by MISSION_ITEM protocol and SET_POSITION_TARGET handler.
    /// Read by navigation_task for target waypoints.
    /// Uses blocking mutex with critical sections for interrupt-safe access.
    pub static MISSION_STORAGE: EmbassyState<MissionStorage> =
        EmbassyState::new(MissionStorage::new_const());

    /// Get current mission state
    ///
    /// Returns the current mission execution state.
    pub fn get_mission_state() -> MissionState {
        MISSION_STATE.with(|state| *state)
    }

    /// Set mission state
    ///
    /// Updates the mission execution state.
    pub fn set_mission_state(state: MissionState) {
        MISSION_STATE.with_mut(|s| *s = state);
    }

    /// Get current navigation target from mission storage
    ///
    /// Returns the current waypoint as PositionTarget if available.
    /// This is the primary interface for navigation_task to get its target.
    pub fn get_current_target() -> Option<PositionTarget> {
        MISSION_STORAGE.with(|storage| storage.current_waypoint().map(PositionTarget::from))
    }

    /// Advance to next waypoint in AUTO mode
    ///
    /// Increments current_index if more waypoints exist and autocontinue is enabled.
    /// Returns true if advanced, false if at last waypoint (mission complete) or autocontinue is disabled.
    pub fn advance_waypoint() -> bool {
        MISSION_STORAGE.with_mut(|storage| {
            let current = storage.current_index();
            let count = storage.count();

            // Check autocontinue of current waypoint
            if let Some(wp) = storage.get_waypoint(current) {
                if wp.autocontinue == 0 {
                    return false; // Don't advance if autocontinue is disabled
                }
            }

            if current + 1 < count {
                let _ = storage.set_current_index(current + 1);
                true
            } else {
                false
            }
        })
    }

    /// Check if mission storage has waypoints
    pub fn has_waypoints() -> bool {
        MISSION_STORAGE.with(|storage| !storage.is_empty())
    }

    /// Clear mission storage and reset state
    pub fn clear_mission() {
        MISSION_STORAGE.with_mut(|storage| storage.clear());
        MISSION_STATE.with_mut(|state| *state = MissionState::Idle);
    }

    /// Add waypoint to mission storage
    ///
    /// Used by SET_POSITION_TARGET handler to create single-waypoint mission.
    pub fn set_single_waypoint(waypoint: Waypoint) {
        MISSION_STORAGE.with_mut(|storage| {
            storage.clear();
            let _ = storage.add_waypoint(waypoint);
        });
    }

    /// Start mission from index 0
    ///
    /// Sets current_index to 0 and MissionState to Running.
    /// Used by MISSION_START handler and GUIDED ARM handler.
    pub fn start_mission() -> Result<(), &'static str> {
        let is_empty = MISSION_STORAGE.with_mut(|storage| {
            if storage.is_empty() {
                return true;
            }
            let _ = storage.set_current_index(0);
            false
        });

        if is_empty {
            return Err("Mission empty");
        }

        MISSION_STATE.with_mut(|state| *state = MissionState::Running);
        Ok(())
    }

    /// Start mission from current index
    ///
    /// Sets MissionState to Running if waypoints exist.
    /// Returns true if mission started, false if no waypoints.
    pub fn start_mission_from_current() -> bool {
        let has_waypoints = MISSION_STORAGE.with(|storage| !storage.is_empty());
        if has_waypoints {
            MISSION_STATE.with_mut(|state| *state = MissionState::Running);
            true
        } else {
            false
        }
    }

    /// Start mission from index 0
    ///
    /// Resets current_index to 0 and sets MissionState to Running if waypoints exist.
    /// Returns true if mission started, false if no waypoints.
    pub fn start_mission_from_beginning() -> bool {
        let started = MISSION_STORAGE.with_mut(|storage| {
            if storage.is_empty() {
                return false;
            }
            let _ = storage.set_current_index(0);
            true
        });

        if started {
            MISSION_STATE.with_mut(|state| *state = MissionState::Running);
        }
        started
    }

    /// Stop mission
    ///
    /// Sets MissionState to Idle.
    pub fn stop_mission() {
        MISSION_STATE.with_mut(|state| *state = MissionState::Idle);
    }

    /// Complete mission
    ///
    /// Sets MissionState to Completed.
    pub fn complete_mission() {
        MISSION_STATE.with_mut(|state| *state = MissionState::Completed);
    }

    /// Clear waypoints
    ///
    /// Clears all waypoints from mission storage.
    pub fn clear_waypoints() {
        MISSION_STORAGE.with_mut(|storage| storage.clear());
    }

    /// Add waypoint
    ///
    /// Adds a waypoint to mission storage.
    /// Returns true if added successfully, false if storage is full.
    pub fn add_waypoint(waypoint: Waypoint) -> bool {
        MISSION_STORAGE.with_mut(|storage| storage.add_waypoint(waypoint).is_ok())
    }
}

#[cfg(feature = "embassy")]
pub use embassy_impl::{
    add_waypoint, advance_waypoint, clear_mission, clear_waypoints, complete_mission,
    get_current_target, get_mission_state, has_waypoints, set_mission_state, set_single_waypoint,
    start_mission, start_mission_from_beginning, start_mission_from_current, stop_mission,
    MISSION_STATE, MISSION_STORAGE,
};

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

// ============================================================================
// Unit Tests
// ============================================================================

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
