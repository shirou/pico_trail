//! Mission Management (firmware re-exports)
//!
//! Re-exports core mission types and provides firmware-specific global state
//! management via the `state` module.
//!
//! # Types (from pico_trail_core)
//!
//! - `Waypoint` - Mission waypoint in MAVLink MISSION_ITEM_INT format
//! - `MissionStorage` - Fixed-size waypoint array (max 50 waypoints)
//! - `MissionState` - Mission execution state (Idle/Running/Completed)
//! - `MAX_WAYPOINTS` - Maximum number of waypoints
//!
//! # Global State (firmware-specific)
//!
//! The `state` module provides global mission state and storage accessible
//! from navigation and handlers, protected by EmbassyState.

mod state;

pub use pico_trail_core::mission::{
    CommandStartResult, MissionEvent, MissionExecutor, MissionSequencer, MissionState,
    MissionStorage, Waypoint, MAX_WAYPOINTS,
};

pub use state::{
    add_waypoint, advance_waypoint, clear_mission, clear_waypoints, complete_mission,
    get_current_target, get_mission_state, has_waypoints, push_current_changed, push_item_reached,
    set_mission_state, set_single_waypoint, start_mission, start_mission_from_beginning,
    start_mission_from_current, stop_mission, take_current_changed, take_item_reached,
    MISSION_SEQUENCER, MISSION_STATE, MISSION_STORAGE,
};
