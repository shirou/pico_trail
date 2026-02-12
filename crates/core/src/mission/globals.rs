//! Global Mission State Management
//!
//! Provides mission execution state tracking and global mission storage.
//!
//! # Architecture
//!
//! MissionStorage serves as the single source of truth for waypoint navigation
//! in both GUIDED and AUTO modes. This module provides:
//!
//! - Global MISSION_STORAGE accessible from navigation and handlers
//! - Global MISSION_SEQUENCER for mission command sequencing
//! - Helper functions for synchronized state access
//!
//! Uses `EmbassyState` for synchronized access, which provides
//! sync (non-async) operations via critical sections.

use super::{MissionSequencer, MissionState, MissionStorage, Waypoint};
use crate::navigation::PositionTarget;
use crate::traits::sync::{EmbassyState, SharedState};

/// Queue for pending MISSION_ITEM_REACHED events
///
/// Auto mode pushes waypoint indices when the sequencer emits ItemReached events.
/// The telemetry system drains this queue and converts to MAVLink messages.
static ITEM_REACHED_QUEUE: EmbassyState<heapless::Vec<u16, 4>> =
    EmbassyState::new(heapless::Vec::new());

/// Queue for pending event-driven MISSION_CURRENT events
///
/// Auto mode pushes waypoint indices when the sequencer emits CurrentChanged events.
/// The telemetry system drains this queue and builds immediate MISSION_CURRENT
/// messages, satisfying the < 100 ms latency requirement (NFR-at4uq).
static CURRENT_CHANGED_QUEUE: EmbassyState<heapless::Vec<u16, 4>> =
    EmbassyState::new(heapless::Vec::new());

/// Global mission state (protected by EmbassyState)
///
/// Tracks mission execution status (Idle/Running/Completed).
pub static MISSION_STATE: EmbassyState<MissionState> = EmbassyState::new(MissionState::Idle);

/// Global mission storage (protected by EmbassyState)
///
/// Single source of truth for waypoint navigation.
pub static MISSION_STORAGE: EmbassyState<MissionStorage> =
    EmbassyState::new(MissionStorage::new_const());

/// Global mission sequencer (protected by EmbassyState)
///
/// Owns mission command sequencing logic including dual-slot NAV/DO
/// advancement, hold time management, and mission speed override.
pub static MISSION_SEQUENCER: EmbassyState<MissionSequencer> =
    EmbassyState::new(MissionSequencer::new());

/// Get current mission state
pub fn get_mission_state() -> MissionState {
    MISSION_STATE.with(|state| *state)
}

/// Set mission state
pub fn set_mission_state(state: MissionState) {
    MISSION_STATE.with_mut(|s| *s = state);
}

/// Get current navigation target from mission storage
pub fn get_current_target() -> Option<PositionTarget> {
    MISSION_STORAGE.with(|storage| storage.current_waypoint().map(PositionTarget::from))
}

/// Advance to next waypoint in AUTO mode
///
/// Returns true if advanced, false if at last waypoint or autocontinue disabled.
pub fn advance_waypoint() -> bool {
    MISSION_STORAGE.with_mut(|storage| {
        let current = storage.current_index();
        let count = storage.count();

        if let Some(wp) = storage.get_waypoint(current) {
            if wp.autocontinue == 0 {
                return false;
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

/// Add waypoint to mission storage for single-waypoint mission
pub fn set_single_waypoint(waypoint: Waypoint) {
    MISSION_STORAGE.with_mut(|storage| {
        storage.clear();
        let _ = storage.add_waypoint(waypoint);
    });
}

/// Start mission from index 0
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
pub fn stop_mission() {
    MISSION_STATE.with_mut(|state| *state = MissionState::Idle);
}

/// Complete mission
pub fn complete_mission() {
    MISSION_STATE.with_mut(|state| *state = MissionState::Completed);
}

/// Clear waypoints from mission storage
pub fn clear_waypoints() {
    MISSION_STORAGE.with_mut(|storage| storage.clear());
}

/// Add waypoint to mission storage
///
/// Returns true if added successfully, false if storage is full.
pub fn add_waypoint(waypoint: Waypoint) -> bool {
    MISSION_STORAGE.with_mut(|storage| storage.add_waypoint(waypoint).is_ok())
}

/// Push a waypoint index into the MISSION_ITEM_REACHED queue
pub fn push_item_reached(seq: u16) {
    ITEM_REACHED_QUEUE.with_mut(|queue| {
        let _ = queue.push(seq);
    });
}

/// Drain all pending MISSION_ITEM_REACHED indices
pub fn take_item_reached() -> heapless::Vec<u16, 4> {
    ITEM_REACHED_QUEUE.with_mut(|queue| {
        let result = queue.clone();
        queue.clear();
        result
    })
}

/// Push a waypoint index into the CURRENT_CHANGED queue
pub fn push_current_changed(seq: u16) {
    CURRENT_CHANGED_QUEUE.with_mut(|queue| {
        let _ = queue.push(seq);
    });
}

/// Drain all pending CURRENT_CHANGED indices
pub fn take_current_changed() -> heapless::Vec<u16, 4> {
    CURRENT_CHANGED_QUEUE.with_mut(|queue| {
        let result = queue.clone();
        queue.clear();
        result
    })
}
