//! Mission Management Types
//!
//! Pure data structures for mission waypoint storage and management.
//!
//! # Mission Storage
//!
//! - Fixed-size waypoint array (max 50 waypoints)
//! - In-memory storage (no persistence)
//! - Compatible with MAVLink MISSION_ITEM_INT format
//!
//! # Waypoint Format
//!
//! - Uses MAVLink MISSION_ITEM_INT format (scaled integer coordinates)
//! - Sequence number for ordering
//! - Command type (waypoint, loiter, return to launch, etc.)
//! - Frame of reference (global, relative, etc.)
//!
//! # Note
//!
//! This module contains only pure data types. Global state management
//! and Embassy-specific wrappers are provided by the firmware crate.

pub mod command;
pub mod executor;
pub mod sequencer;
pub mod state;

use heapless::Vec;

pub use command::{cmd_has_location, is_nav_command, MAV_CMD_DO_CHANGE_SPEED, MAV_CMD_NAV_LAST};
pub use executor::{CommandStartResult, MissionEvent, MissionExecutor};
pub use sequencer::MissionSequencer;
pub use state::MissionState;

/// Maximum number of waypoints in a mission
pub const MAX_WAYPOINTS: usize = 50;

/// Mission waypoint
///
/// Represents a single waypoint in a mission plan. Uses MAVLink MISSION_ITEM_INT
/// format with scaled integer coordinates for better precision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Sequence number (0-indexed)
    pub seq: u16,
    /// Frame of reference (MAV_FRAME_GLOBAL_RELATIVE_ALT, etc.)
    pub frame: u8,
    /// Command ID (MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LOITER_UNLIM, etc.)
    pub command: u16,
    /// Current waypoint (0=false, 1=true)
    pub current: u8,
    /// Autocontinue to next waypoint (0=false, 1=true)
    pub autocontinue: u8,
    /// PARAM1 (command-specific, e.g., hold time for loiter)
    pub param1: f32,
    /// PARAM2 (command-specific, e.g., acceptance radius)
    pub param2: f32,
    /// PARAM3 (command-specific, e.g., pass through waypoint)
    pub param3: f32,
    /// PARAM4 (command-specific, e.g., desired yaw angle)
    pub param4: f32,
    /// X coordinate (latitude in degrees * 1e7)
    pub x: i32,
    /// Y coordinate (longitude in degrees * 1e7)
    pub y: i32,
    /// Z coordinate (altitude in meters)
    pub z: f32,
}

impl Default for Waypoint {
    fn default() -> Self {
        Self {
            seq: 0,
            frame: 0,
            command: 0,
            current: 0,
            autocontinue: 1,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 0.0,
        }
    }
}

impl Waypoint {
    /// Create a new waypoint with given coordinates
    ///
    /// # Arguments
    ///
    /// * `seq` - Sequence number
    /// * `lat` - Latitude in degrees * 1e7
    /// * `lon` - Longitude in degrees * 1e7
    /// * `alt` - Altitude in meters
    pub fn new(seq: u16, lat: i32, lon: i32, alt: f32) -> Self {
        Self {
            seq,
            frame: 3,    // MAV_FRAME_GLOBAL_RELATIVE_ALT
            command: 16, // MAV_CMD_NAV_WAYPOINT
            current: 0,
            autocontinue: 1,
            param1: 0.0,
            param2: 5.0, // 5m acceptance radius
            param3: 0.0,
            param4: 0.0,
            x: lat,
            y: lon,
            z: alt,
        }
    }

    /// Get latitude in degrees
    pub fn latitude(&self) -> f64 {
        self.x as f64 / 1e7
    }

    /// Get longitude in degrees
    pub fn longitude(&self) -> f64 {
        self.y as f64 / 1e7
    }

    /// Get altitude in meters
    pub fn altitude(&self) -> f32 {
        self.z
    }
}

/// Mission storage
///
/// Stores mission waypoints in a fixed-size array. Supports adding, retrieving,
/// and clearing waypoints.
#[derive(Debug, Clone)]
pub struct MissionStorage {
    /// Waypoint array (max 50 waypoints)
    waypoints: Vec<Waypoint, MAX_WAYPOINTS>,
    /// Current waypoint index (for navigation)
    current_index: u16,
}

impl Default for MissionStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl MissionStorage {
    /// Create a new empty mission storage
    pub fn new() -> Self {
        Self {
            waypoints: Vec::new(),
            current_index: 0,
        }
    }

    /// Create a new empty mission storage (const fn for static initialization)
    ///
    /// This is required for static MISSION_STORAGE initialization.
    pub const fn new_const() -> Self {
        Self {
            waypoints: Vec::new(),
            current_index: 0,
        }
    }

    /// Get number of waypoints
    pub fn count(&self) -> u16 {
        self.waypoints.len() as u16
    }

    /// Check if mission is empty
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    /// Clear all waypoints
    pub fn clear(&mut self) {
        self.waypoints.clear();
        self.current_index = 0;
    }

    /// Add a waypoint to the mission
    ///
    /// Returns Ok if waypoint was added, or Err if mission is full.
    pub fn add_waypoint(&mut self, waypoint: Waypoint) -> Result<(), &'static str> {
        self.waypoints
            .push(waypoint)
            .map_err(|_| "Mission full (max 50 waypoints)")
    }

    /// Get a waypoint by sequence number
    ///
    /// Returns Some(waypoint) if found, None otherwise.
    pub fn get_waypoint(&self, seq: u16) -> Option<&Waypoint> {
        self.waypoints.get(seq as usize)
    }

    /// Get mutable waypoint by sequence number
    pub fn get_waypoint_mut(&mut self, seq: u16) -> Option<&mut Waypoint> {
        self.waypoints.get_mut(seq as usize)
    }

    /// Set waypoint at specific sequence number
    ///
    /// Returns Ok if waypoint was set, or Err if index out of bounds.
    pub fn set_waypoint(&mut self, seq: u16, waypoint: Waypoint) -> Result<(), &'static str> {
        if (seq as usize) < self.waypoints.len() {
            self.waypoints[seq as usize] = waypoint;
            Ok(())
        } else {
            Err("Waypoint index out of bounds")
        }
    }

    /// Get current waypoint index
    pub fn current_index(&self) -> u16 {
        self.current_index
    }

    /// Set current waypoint index
    ///
    /// Returns Ok if index is valid, or Err if out of bounds.
    pub fn set_current_index(&mut self, index: u16) -> Result<(), &'static str> {
        if (index as usize) < self.waypoints.len() {
            self.current_index = index;
            Ok(())
        } else {
            Err("Waypoint index out of bounds")
        }
    }

    /// Get current waypoint
    pub fn current_waypoint(&self) -> Option<&Waypoint> {
        self.waypoints.get(self.current_index as usize)
    }

    /// Reserve space for N waypoints
    ///
    /// Clears existing mission and prepares for upload of N waypoints.
    pub fn reserve(&mut self, _count: u16) {
        self.clear();
        // Note: heapless::Vec doesn't have reserve(), but we clear to prepare
    }

    /// Get all waypoints as slice
    pub fn waypoints(&self) -> &[Waypoint] {
        &self.waypoints
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_waypoint_creation() {
        let wp = Waypoint::new(0, 370000000, -1220000000, 100.0);
        assert_eq!(wp.seq, 0);
        assert_eq!(wp.x, 370000000);
        assert_eq!(wp.y, -1220000000);
        assert_eq!(wp.z, 100.0);
        assert_eq!(wp.frame, 3); // GLOBAL_RELATIVE_ALT
        assert_eq!(wp.command, 16); // NAV_WAYPOINT
    }

    #[test]
    fn test_waypoint_coordinates() {
        let wp = Waypoint::new(0, 370000000, -1220000000, 100.0);
        assert!((wp.latitude() - 37.0).abs() < 0.0001);
        assert!((wp.longitude() - (-122.0)).abs() < 0.0001);
        assert_eq!(wp.altitude(), 100.0);
    }

    #[test]
    fn test_mission_storage_creation() {
        let storage = MissionStorage::new();
        assert_eq!(storage.count(), 0);
        assert!(storage.is_empty());
    }

    #[test]
    fn test_add_waypoint() {
        let mut storage = MissionStorage::new();
        let wp1 = Waypoint::new(0, 370000000, -1220000000, 100.0);
        let wp2 = Waypoint::new(1, 370010000, -1220010000, 120.0);

        assert!(storage.add_waypoint(wp1).is_ok());
        assert_eq!(storage.count(), 1);

        assert!(storage.add_waypoint(wp2).is_ok());
        assert_eq!(storage.count(), 2);
    }

    #[test]
    fn test_get_waypoint() {
        let mut storage = MissionStorage::new();
        let wp = Waypoint::new(0, 370000000, -1220000000, 100.0);
        storage.add_waypoint(wp).unwrap();

        let retrieved = storage.get_waypoint(0).unwrap();
        assert_eq!(retrieved.seq, 0);
        assert_eq!(retrieved.x, 370000000);
    }

    #[test]
    fn test_get_waypoint_not_found() {
        let storage = MissionStorage::new();
        assert!(storage.get_waypoint(0).is_none());
        assert!(storage.get_waypoint(10).is_none());
    }

    #[test]
    fn test_clear_mission() {
        let mut storage = MissionStorage::new();
        let wp1 = Waypoint::new(0, 370000000, -1220000000, 100.0);
        let wp2 = Waypoint::new(1, 370010000, -1220010000, 120.0);

        storage.add_waypoint(wp1).unwrap();
        storage.add_waypoint(wp2).unwrap();
        assert_eq!(storage.count(), 2);

        storage.clear();
        assert_eq!(storage.count(), 0);
        assert!(storage.is_empty());
    }

    #[test]
    fn test_current_waypoint() {
        let mut storage = MissionStorage::new();
        let wp1 = Waypoint::new(0, 370000000, -1220000000, 100.0);
        let wp2 = Waypoint::new(1, 370010000, -1220010000, 120.0);

        storage.add_waypoint(wp1).unwrap();
        storage.add_waypoint(wp2).unwrap();

        assert_eq!(storage.current_index(), 0);
        let current = storage.current_waypoint().unwrap();
        assert_eq!(current.seq, 0);

        storage.set_current_index(1).unwrap();
        assert_eq!(storage.current_index(), 1);
        let current = storage.current_waypoint().unwrap();
        assert_eq!(current.seq, 1);
    }

    #[test]
    fn test_set_current_index_out_of_bounds() {
        let mut storage = MissionStorage::new();
        let wp = Waypoint::new(0, 370000000, -1220000000, 100.0);
        storage.add_waypoint(wp).unwrap();

        assert!(storage.set_current_index(0).is_ok());
        assert!(storage.set_current_index(1).is_err());
        assert!(storage.set_current_index(10).is_err());
    }

    #[test]
    fn test_mission_full() {
        let mut storage = MissionStorage::new();

        // Add 50 waypoints (max)
        for i in 0..MAX_WAYPOINTS {
            let wp = Waypoint::new(i as u16, 370000000, -1220000000, 100.0);
            assert!(storage.add_waypoint(wp).is_ok());
        }

        assert_eq!(storage.count(), MAX_WAYPOINTS as u16);

        // Try to add one more (should fail)
        let wp = Waypoint::new(MAX_WAYPOINTS as u16, 370000000, -1220000000, 100.0);
        assert!(storage.add_waypoint(wp).is_err());
    }

    #[test]
    fn test_set_waypoint() {
        let mut storage = MissionStorage::new();
        let wp1 = Waypoint::new(0, 370000000, -1220000000, 100.0);
        storage.add_waypoint(wp1).unwrap();

        // Update waypoint at index 0
        let wp2 = Waypoint::new(0, 370010000, -1220010000, 120.0);
        assert!(storage.set_waypoint(0, wp2).is_ok());

        let retrieved = storage.get_waypoint(0).unwrap();
        assert_eq!(retrieved.x, 370010000);
        assert_eq!(retrieved.z, 120.0);
    }

    #[test]
    fn test_reserve() {
        let mut storage = MissionStorage::new();
        let wp = Waypoint::new(0, 370000000, -1220000000, 100.0);
        storage.add_waypoint(wp).unwrap();
        assert_eq!(storage.count(), 1);

        storage.reserve(10);
        assert_eq!(storage.count(), 0);
    }
}
