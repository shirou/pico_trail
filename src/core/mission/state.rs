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
//! - Helper functions for async state access

#[cfg(feature = "embassy")]
use super::MissionStorage;
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
// Global State (embassy feature)
// ============================================================================

#[cfg(feature = "embassy")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "embassy")]
use embassy_sync::mutex::Mutex;

/// Global mission state (protected by Mutex)
///
/// Tracks mission execution status (Idle/Running/Completed).
/// Updated by mode handlers and navigation task.
#[cfg(feature = "embassy")]
pub static MISSION_STATE: Mutex<CriticalSectionRawMutex, MissionState> =
    Mutex::new(MissionState::Idle);

/// Global mission storage (protected by Mutex)
///
/// Single source of truth for waypoint navigation.
/// Updated by MISSION_ITEM protocol and SET_POSITION_TARGET handler.
/// Read by navigation_task for target waypoints.
#[cfg(feature = "embassy")]
pub static MISSION_STORAGE: Mutex<CriticalSectionRawMutex, MissionStorage> =
    Mutex::new(MissionStorage::new_const());

// ============================================================================
// Helper Functions (embassy feature)
// ============================================================================

/// Get current mission state
///
/// Returns the current mission execution state.
#[cfg(feature = "embassy")]
pub async fn get_mission_state() -> MissionState {
    *MISSION_STATE.lock().await
}

/// Set mission state
///
/// Updates the mission execution state.
#[cfg(feature = "embassy")]
pub async fn set_mission_state(state: MissionState) {
    *MISSION_STATE.lock().await = state;
}

/// Get current navigation target from mission storage
///
/// Returns the current waypoint as PositionTarget if available.
/// This is the primary interface for navigation_task to get its target.
#[cfg(feature = "embassy")]
pub async fn get_current_target() -> Option<PositionTarget> {
    let storage = MISSION_STORAGE.lock().await;
    storage.current_waypoint().map(PositionTarget::from)
}

/// Advance to next waypoint in AUTO mode
///
/// Increments current_index if more waypoints exist and autocontinue is enabled.
/// Returns true if advanced, false if at last waypoint (mission complete) or autocontinue is disabled.
#[cfg(feature = "embassy")]
pub async fn advance_waypoint() -> bool {
    let mut storage = MISSION_STORAGE.lock().await;
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
}

/// Check if mission storage has waypoints
#[cfg(feature = "embassy")]
pub async fn has_waypoints() -> bool {
    !MISSION_STORAGE.lock().await.is_empty()
}

/// Clear mission storage and reset state
#[cfg(feature = "embassy")]
pub async fn clear_mission() {
    MISSION_STORAGE.lock().await.clear();
    *MISSION_STATE.lock().await = MissionState::Idle;
}

/// Add waypoint to mission storage
///
/// Used by SET_POSITION_TARGET handler to create single-waypoint mission.
#[cfg(feature = "embassy")]
pub async fn set_single_waypoint(waypoint: Waypoint) {
    let mut storage = MISSION_STORAGE.lock().await;
    storage.clear();
    let _ = storage.add_waypoint(waypoint);
}

/// Start mission from index 0
///
/// Sets current_index to 0 and MissionState to Running.
/// Used by MISSION_START handler and GUIDED ARM handler.
#[cfg(feature = "embassy")]
pub async fn start_mission() -> Result<(), &'static str> {
    let mut storage = MISSION_STORAGE.lock().await;
    if storage.is_empty() {
        return Err("Mission empty");
    }
    let _ = storage.set_current_index(0);
    drop(storage);

    *MISSION_STATE.lock().await = MissionState::Running;
    Ok(())
}

// ============================================================================
// Synchronous Access (for command handlers using critical_section)
// ============================================================================

/// Synchronous mission state storage (for use with critical_section)
///
/// This is a separate storage from the async MISSION_STATE/MISSION_STORAGE
/// to allow synchronous access from command handlers.
#[cfg(feature = "embassy")]
pub static MISSION_STATE_SYNC: critical_section::Mutex<core::cell::RefCell<MissionState>> =
    critical_section::Mutex::new(core::cell::RefCell::new(MissionState::Idle));

/// Get mission state synchronously (for command handlers)
#[cfg(feature = "embassy")]
pub fn get_mission_state_sync() -> MissionState {
    critical_section::with(|cs| *MISSION_STATE_SYNC.borrow_ref(cs))
}

/// Set mission state synchronously (for command handlers)
#[cfg(feature = "embassy")]
pub fn set_mission_state_sync(state: MissionState) {
    critical_section::with(|cs| {
        *MISSION_STATE_SYNC.borrow_ref_mut(cs) = state;
    });
}

/// Check if mission storage has waypoints (synchronous, for command handlers)
///
/// Uses the async MISSION_STORAGE via try_lock to check without blocking.
/// Returns false if lock cannot be acquired (assumes no waypoints in that case).
#[cfg(feature = "embassy")]
pub fn has_waypoints_sync() -> bool {
    // Access the async MISSION_STORAGE using try_lock
    // If lock is held, we conservatively return false
    if let Ok(storage) = MISSION_STORAGE.try_lock() {
        !storage.is_empty()
    } else {
        false
    }
}

/// Start mission synchronously (for GUIDED mode ARM handler)
///
/// Sets MissionState to Running if waypoints exist.
/// Returns true if mission started, false if no waypoints.
#[cfg(feature = "embassy")]
pub fn start_mission_sync() -> bool {
    let has_waypoints = has_waypoints_sync();
    if has_waypoints {
        set_mission_state_sync(MissionState::Running);
        true
    } else {
        false
    }
}

/// Start mission from index 0 synchronously (for AUTO mode MISSION_START handler)
///
/// Resets current_index to 0 and sets MissionState to Running if waypoints exist.
/// Returns true if mission started, false if no waypoints or lock unavailable.
#[cfg(feature = "embassy")]
pub fn start_mission_from_beginning_sync() -> bool {
    // Try to acquire the storage lock
    if let Ok(mut storage) = MISSION_STORAGE.try_lock() {
        if storage.is_empty() {
            return false;
        }
        let _ = storage.set_current_index(0);
        drop(storage);
        set_mission_state_sync(MissionState::Running);
        true
    } else {
        // Lock unavailable - return false
        false
    }
}

/// Stop mission synchronously (for mode change handling)
///
/// Sets MissionState to Idle.
#[cfg(feature = "embassy")]
pub fn stop_mission_sync() {
    set_mission_state_sync(MissionState::Idle);
}

/// Complete mission synchronously (for waypoint arrival handling)
///
/// Sets MissionState to Completed.
#[cfg(feature = "embassy")]
pub fn complete_mission_sync() {
    set_mission_state_sync(MissionState::Completed);
}

/// Clear waypoints synchronously (for mission handlers)
///
/// Uses try_lock to access MISSION_STORAGE without blocking.
/// Returns true if cleared successfully, false if lock unavailable.
#[cfg(feature = "embassy")]
pub fn clear_waypoints_sync() -> bool {
    if let Ok(mut storage) = MISSION_STORAGE.try_lock() {
        storage.clear();
        true
    } else {
        false
    }
}

/// Add waypoint synchronously (for mission handlers)
///
/// Uses try_lock to access MISSION_STORAGE without blocking.
/// Returns true if added successfully, false if lock unavailable or storage error.
#[cfg(feature = "embassy")]
pub fn add_waypoint_sync(waypoint: Waypoint) -> bool {
    if let Ok(mut storage) = MISSION_STORAGE.try_lock() {
        storage.add_waypoint(waypoint).is_ok()
    } else {
        false
    }
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
