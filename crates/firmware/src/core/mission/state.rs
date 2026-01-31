//! Mission State Management
//!
//! Provides mission execution state tracking and global mission storage.
//!
//! # Architecture
//!
//! MissionStorage serves as the single source of truth for waypoint navigation
//! in both GUIDED and AUTO modes. This module provides:
//!
//! - Global MISSION_STORAGE accessible from navigation and handlers
//! - Helper functions for synchronized state access
//!
//! # Types
//!
//! MissionState enum is defined in pico_trail_core and re-exported here.
//!
//! # Migration Note (FR-jpmdj)
//!
//! This module uses `EmbassyState` for synchronized access, which provides
//! sync (non-async) operations via critical sections. All functions are now
//! synchronous, eliminating the need for separate `_sync` variants.

use super::{MissionState, MissionStorage, Waypoint};
use crate::core::traits::{EmbassyState, SharedState};
use crate::subsystems::navigation::PositionTarget;
use pico_trail_core::mission::MissionSequencer;

/// Queue for pending MISSION_ITEM_REACHED events
///
/// Auto mode pushes waypoint indices when the sequencer emits ItemReached events.
/// The telemetry system drains this queue and converts to MAVLink messages.
/// Protected by EmbassyState for interrupt-safe access.
static ITEM_REACHED_QUEUE: EmbassyState<heapless::Vec<u16, 4>> =
    EmbassyState::new(heapless::Vec::new());

/// Queue for pending event-driven MISSION_CURRENT events
///
/// Auto mode pushes waypoint indices when the sequencer emits CurrentChanged events.
/// The telemetry system drains this queue and builds immediate MISSION_CURRENT
/// messages, satisfying the < 100 ms latency requirement (NFR-at4uq).
static CURRENT_CHANGED_QUEUE: EmbassyState<heapless::Vec<u16, 4>> =
    EmbassyState::new(heapless::Vec::new());

// ============================================================================
// Global State and Helper Functions
// ============================================================================

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

/// Global mission sequencer (protected by EmbassyState)
///
/// Owns mission command sequencing logic including dual-slot NAV/DO
/// advancement, hold time management, and mission speed override.
/// Accessed by Auto mode for mission execution and by telemetry for
/// MISSION_CURRENT reporting.
pub static MISSION_SEQUENCER: EmbassyState<MissionSequencer> =
    EmbassyState::new(MissionSequencer::new());

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

/// Push a waypoint index into the MISSION_ITEM_REACHED queue
///
/// Called by Auto mode when the sequencer emits an ItemReached event.
/// Silently drops if queue is full (4 entries max).
pub fn push_item_reached(seq: u16) {
    ITEM_REACHED_QUEUE.with_mut(|queue| {
        let _ = queue.push(seq);
    });
}

/// Drain all pending MISSION_ITEM_REACHED indices
///
/// Called by the telemetry system to get pending events for conversion
/// to MAVLink MISSION_ITEM_REACHED messages.
pub fn take_item_reached() -> heapless::Vec<u16, 4> {
    ITEM_REACHED_QUEUE.with_mut(|queue| {
        let result = queue.clone();
        queue.clear();
        result
    })
}

/// Push a waypoint index into the CURRENT_CHANGED queue for immediate MISSION_CURRENT
///
/// Called by Auto mode when the sequencer emits a CurrentChanged event.
/// Silently drops if queue is full (4 entries max).
pub fn push_current_changed(seq: u16) {
    CURRENT_CHANGED_QUEUE.with_mut(|queue| {
        let _ = queue.push(seq);
    });
}

/// Drain all pending CURRENT_CHANGED indices
///
/// Called by the telemetry system to get pending events for conversion
/// to immediate MISSION_CURRENT MAVLink messages (NFR-at4uq).
pub fn take_current_changed() -> heapless::Vec<u16, 4> {
    CURRENT_CHANGED_QUEUE.with_mut(|queue| {
        let result = queue.clone();
        queue.clear();
        result
    })
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mission_state_reexport() {
        // Verify re-exported MissionState works
        let state = MissionState::default();
        assert_eq!(state, MissionState::Idle);
    }

    #[test]
    fn test_waypoint_to_position_target_via_core() {
        // Verify From<&Waypoint> for PositionTarget from core works
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
}
