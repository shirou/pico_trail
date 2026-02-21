//! Mission Management (firmware re-exports)
//!
//! Re-exports all mission types, state management, and global storage from
//! pico_trail_core. There is a single set of global statics (MISSION_STATE,
//! MISSION_STORAGE, MISSION_SEQUENCER) defined in the core crate, ensuring
//! handlers and navigation tasks operate on the same data.

pub use pico_trail_core::mission::{
    add_waypoint, advance_waypoint, clear_mission, clear_waypoints, complete_mission,
    get_current_target, get_mission_state, has_waypoints, push_current_changed, push_item_reached,
    set_mission_state, set_single_waypoint, start_mission, start_mission_from_beginning,
    start_mission_from_current, stop_mission, take_current_changed, take_item_reached,
    CommandStartResult, MissionEvent, MissionExecutor, MissionSequencer, MissionState,
    MissionStorage, Waypoint, MAX_WAYPOINTS, MISSION_SEQUENCER, MISSION_STATE, MISSION_STORAGE,
};
