//! Mission Protocol Handler
//!
//! Handles mission upload and download via MAVLink protocol.
//!
//! # Mission Upload Flow (GCS → Autopilot)
//!
//! 1. GCS sends MISSION_COUNT with number of waypoints
//! 2. Autopilot responds with MISSION_REQUEST_INT for seq=0
//! 3. GCS sends MISSION_ITEM_INT for seq=0
//! 4. Autopilot responds with MISSION_REQUEST_INT for seq=1
//! 5. ... repeat until all waypoints received
//! 6. Autopilot sends MISSION_ACK with result
//!
//! # Mission Download Flow (Autopilot → GCS)
//!
//! 1. GCS sends MISSION_REQUEST_LIST
//! 2. Autopilot responds with MISSION_COUNT
//! 3. GCS sends MISSION_REQUEST_INT for seq=0
//! 4. Autopilot responds with MISSION_ITEM_INT for seq=0
//! 5. ... repeat until all waypoints sent
//! 6. GCS sends MISSION_ACK
//!
//! # Timeout Handling
//!
//! - Upload/download aborts if no message received within 5 seconds
//! - State machine returns to Idle on timeout
//! - Partial missions are discarded on timeout

use crate::core::mission::{MissionStorage, Waypoint};
use mavlink::common::{
    MavCmd, MavFrame, MavMissionResult, MavMissionType, MISSION_ACK_DATA, MISSION_COUNT_DATA,
    MISSION_ITEM_INT_DATA, MISSION_REQUEST_INT_DATA, MISSION_REQUEST_LIST_DATA,
};

/// Mission transfer timeout (5 seconds in microseconds)
const MISSION_TIMEOUT_US: u64 = 5_000_000;

/// Mission transfer state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionState {
    /// No mission transfer in progress
    Idle,
    /// Upload in progress (GCS → Autopilot)
    UploadInProgress {
        /// Total waypoint count
        count: u16,
        /// Next expected sequence number
        next_seq: u16,
        /// Last activity timestamp (microseconds)
        last_activity_us: u64,
        /// Sender's system ID for ACK response
        sender_system_id: u8,
        /// Sender's component ID for ACK response
        sender_component_id: u8,
    },
    /// Download in progress (Autopilot → GCS)
    DownloadInProgress {
        /// Last activity timestamp (microseconds)
        last_activity_us: u64,
        /// Sender's system ID for ACK response
        sender_system_id: u8,
        /// Sender's component ID for ACK response
        sender_component_id: u8,
    },
}

impl Default for MissionState {
    fn default() -> Self {
        Self::Idle
    }
}

/// Mission protocol handler
///
/// Manages mission upload/download and state machine.
pub struct MissionHandler {
    /// Mission storage
    storage: MissionStorage,
    /// Current transfer state
    state: MissionState,
    /// System ID (for message generation)
    #[allow(dead_code)]
    system_id: u8,
    /// Component ID (for message generation)
    #[allow(dead_code)]
    component_id: u8,
}

impl MissionHandler {
    /// Create a new mission handler
    ///
    /// # Arguments
    ///
    /// * `system_id` - MAVLink system ID
    /// * `component_id` - MAVLink component ID
    pub fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            storage: MissionStorage::new(),
            state: MissionState::Idle,
            system_id,
            component_id,
        }
    }

    /// Get mission storage reference
    pub fn storage(&self) -> &MissionStorage {
        &self.storage
    }

    /// Get mutable mission storage reference
    pub fn storage_mut(&mut self) -> &mut MissionStorage {
        &mut self.storage
    }

    /// Get current mission state
    pub fn state(&self) -> MissionState {
        self.state
    }

    /// Check for timeout and abort transfer if necessary
    ///
    /// # Arguments
    ///
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// Returns Some((sender_system_id, sender_component_id)) if timeout occurred, None otherwise
    pub fn check_timeout(&mut self, current_time_us: u64) -> Option<(u8, u8)> {
        match self.state {
            MissionState::UploadInProgress {
                last_activity_us,
                sender_system_id,
                sender_component_id,
                ..
            }
            | MissionState::DownloadInProgress {
                last_activity_us,
                sender_system_id,
                sender_component_id,
            } => {
                if current_time_us - last_activity_us > MISSION_TIMEOUT_US {
                    crate::log_warn!("Mission transfer timeout, aborting");
                    self.abort_transfer();
                    Some((sender_system_id, sender_component_id))
                } else {
                    None
                }
            }
            MissionState::Idle => None,
        }
    }

    /// Abort current mission transfer
    fn abort_transfer(&mut self) {
        match self.state {
            MissionState::UploadInProgress { .. } => {
                // Discard partial mission
                self.storage.clear();
            }
            MissionState::DownloadInProgress { .. } => {
                // Download abort doesn't affect stored mission
            }
            MissionState::Idle => {}
        }
        self.state = MissionState::Idle;
    }

    /// Handle MISSION_REQUEST_LIST message
    ///
    /// Initiates mission download from autopilot to GCS.
    ///
    /// # Arguments
    ///
    /// * `_data` - The MISSION_REQUEST_LIST message data
    /// * `current_time_us` - Current timestamp in microseconds
    /// * `sender_system_id` - System ID of the sender (from MAVLink header)
    /// * `sender_component_id` - Component ID of the sender (from MAVLink header)
    pub fn handle_request_list(
        &mut self,
        _data: &MISSION_REQUEST_LIST_DATA,
        current_time_us: u64,
        sender_system_id: u8,
        sender_component_id: u8,
    ) -> MISSION_COUNT_DATA {
        crate::log_info!("Mission download requested");
        self.state = MissionState::DownloadInProgress {
            last_activity_us: current_time_us,
            sender_system_id,
            sender_component_id,
        };

        MISSION_COUNT_DATA {
            target_system: sender_system_id,       // Reply to sender
            target_component: sender_component_id, // Reply to sender
            count: self.storage.count(),
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION, // MAVLink v2 extension
            opaque_id: 0,                                           // MAVLink v2 extension
        }
    }

    /// Handle MISSION_REQUEST_INT message
    ///
    /// Sends requested waypoint to GCS during download.
    pub fn handle_request_int(
        &mut self,
        data: &MISSION_REQUEST_INT_DATA,
        current_time_us: u64,
    ) -> Result<MISSION_ITEM_INT_DATA, MavMissionResult> {
        // Update activity timestamp while preserving sender IDs
        if let MissionState::DownloadInProgress {
            last_activity_us,
            sender_system_id,
            sender_component_id,
        } = &mut self.state
        {
            *last_activity_us = current_time_us;
            let _ = (*sender_system_id, *sender_component_id); // keep sender IDs
        } else {
            crate::log_warn!("Received MISSION_REQUEST_INT while not downloading");
            return Err(MavMissionResult::MAV_MISSION_ERROR);
        }

        let seq = data.seq;
        match self.storage.get_waypoint(seq) {
            Some(wp) => {
                crate::log_debug!("Sending waypoint {}", seq);
                Ok(self.waypoint_to_mission_item(wp, data.target_system, data.target_component))
            }
            None => {
                crate::log_warn!("Waypoint {} not found", seq);
                Err(MavMissionResult::MAV_MISSION_INVALID_SEQUENCE)
            }
        }
    }

    /// Handle MISSION_COUNT message
    ///
    /// Initiates mission upload from GCS to autopilot.
    ///
    /// # Arguments
    ///
    /// * `data` - The MISSION_COUNT message data
    /// * `current_time_us` - Current timestamp in microseconds
    /// * `sender_system_id` - System ID of the sender (from MAVLink header)
    /// * `sender_component_id` - Component ID of the sender (from MAVLink header)
    pub fn handle_count(
        &mut self,
        data: &MISSION_COUNT_DATA,
        current_time_us: u64,
        sender_system_id: u8,
        sender_component_id: u8,
    ) -> MISSION_REQUEST_INT_DATA {
        let count = data.count;
        crate::log_info!("Mission upload started: {} waypoints", count);

        // Clear existing mission and prepare for upload
        self.storage.reserve(count);

        self.state = MissionState::UploadInProgress {
            count,
            next_seq: 0,
            last_activity_us: current_time_us,
            sender_system_id,
            sender_component_id,
        };

        // Request first waypoint - reply to sender
        MISSION_REQUEST_INT_DATA {
            target_system: sender_system_id,
            target_component: sender_component_id,
            seq: 0,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION, // MAVLink v2 extension
        }
    }

    /// Handle MISSION_ITEM_INT message
    ///
    /// Receives waypoint from GCS during upload.
    pub fn handle_item_int(
        &mut self,
        data: &MISSION_ITEM_INT_DATA,
        current_time_us: u64,
    ) -> Result<Option<MISSION_REQUEST_INT_DATA>, MavMissionResult> {
        match self.state {
            MissionState::UploadInProgress {
                count,
                next_seq,
                last_activity_us: _,
                sender_system_id,
                sender_component_id,
            } => {
                // Verify sequence number
                if data.seq != next_seq {
                    crate::log_warn!(
                        "Waypoint sequence mismatch: expected {}, got {}",
                        next_seq,
                        data.seq
                    );
                    self.abort_transfer();
                    return Err(MavMissionResult::MAV_MISSION_INVALID_SEQUENCE);
                }

                // Convert and store waypoint
                let wp = self.mission_item_to_waypoint(data);
                if let Err(_e) = self.storage.add_waypoint(wp) {
                    crate::log_warn!("Failed to add waypoint");
                    self.abort_transfer();
                    return Err(MavMissionResult::MAV_MISSION_ERROR);
                }

                crate::log_debug!("Received waypoint {}/{}", next_seq + 1, count);

                // Update state - preserve sender IDs
                let new_next_seq = next_seq + 1;
                self.state = MissionState::UploadInProgress {
                    count,
                    next_seq: new_next_seq,
                    last_activity_us: current_time_us,
                    sender_system_id,
                    sender_component_id,
                };

                // Check if upload complete
                if new_next_seq >= count {
                    crate::log_info!("Mission upload complete: {} waypoints", count);
                    self.state = MissionState::Idle;
                    Ok(None) // No more requests, upload complete
                } else {
                    // Request next waypoint - reply to sender
                    Ok(Some(MISSION_REQUEST_INT_DATA {
                        target_system: sender_system_id,
                        target_component: sender_component_id,
                        seq: new_next_seq,
                        mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION, // MAVLink v2 extension
                    }))
                }
            }
            MissionState::Idle => {
                // Handle "Fly Here" command from Mission Planner
                // This sends a single MISSION_ITEM directly without MISSION_COUNT
                if data.seq == 0 {
                    crate::log_info!("Fly Here: single waypoint received");
                    // Clear existing mission and store the single waypoint
                    self.storage.clear();
                    self.storage.reserve(1);
                    let wp = self.mission_item_to_waypoint(data);
                    if let Err(_e) = self.storage.add_waypoint(wp) {
                        crate::log_warn!("Failed to add waypoint");
                        return Err(MavMissionResult::MAV_MISSION_ERROR);
                    }
                    // Return None to signal completion (sends MISSION_ACK with ACCEPTED)
                    Ok(None)
                } else {
                    crate::log_warn!(
                        "Received MISSION_ITEM_INT seq={} while idle (expected 0)",
                        data.seq
                    );
                    Err(MavMissionResult::MAV_MISSION_INVALID_SEQUENCE)
                }
            }
            MissionState::DownloadInProgress { .. } => {
                crate::log_warn!("Received MISSION_ITEM_INT while downloading");
                Err(MavMissionResult::MAV_MISSION_ERROR)
            }
        }
    }

    /// Handle MISSION_ACK message
    ///
    /// Completes mission download.
    pub fn handle_ack(&mut self, _data: &MISSION_ACK_DATA) {
        match self.state {
            MissionState::DownloadInProgress { .. } => {
                crate::log_info!("Mission download acknowledged");
                self.state = MissionState::Idle;
            }
            _ => {
                crate::log_warn!("Received MISSION_ACK while not in download");
            }
        }
    }

    /// Convert Waypoint to MISSION_ITEM_INT_DATA
    fn waypoint_to_mission_item(
        &self,
        wp: &Waypoint,
        target_system: u8,
        target_component: u8,
    ) -> MISSION_ITEM_INT_DATA {
        // Use default values if conversion fails
        let frame = match wp.frame {
            3 => MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            _ => MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT, // Default fallback
        };
        let command = match wp.command {
            16 => MavCmd::MAV_CMD_NAV_WAYPOINT,
            _ => MavCmd::MAV_CMD_NAV_WAYPOINT, // Default fallback
        };

        MISSION_ITEM_INT_DATA {
            target_system,
            target_component,
            seq: wp.seq,
            frame,
            command,
            current: wp.current,
            autocontinue: wp.autocontinue,
            param1: wp.param1,
            param2: wp.param2,
            param3: wp.param3,
            param4: wp.param4,
            x: wp.x,
            y: wp.y,
            z: wp.z,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION, // MAVLink v2 extension
        }
    }

    /// Convert MISSION_ITEM_INT_DATA to Waypoint
    fn mission_item_to_waypoint(&self, data: &MISSION_ITEM_INT_DATA) -> Waypoint {
        Waypoint {
            seq: data.seq,
            frame: data.frame as u8,
            command: data.command as u16,
            current: data.current,
            autocontinue: data.autocontinue,
            param1: data.param1,
            param2: data.param2,
            param3: data.param3,
            param4: data.param4,
            x: data.x,
            y: data.y,
            z: data.z,
        }
    }

    /// Create MISSION_ACK message
    pub fn create_ack(
        &self,
        result: MavMissionResult,
        target_system: u8,
        target_component: u8,
    ) -> MISSION_ACK_DATA {
        MISSION_ACK_DATA {
            target_system,
            target_component,
            mavtype: result,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION, // MAVLink v2 extension
            opaque_id: 0,                                           // MAVLink v2 extension
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_handler_creation() {
        let handler = MissionHandler::new(1, 1);
        assert_eq!(handler.storage().count(), 0);
        assert_eq!(handler.state(), MissionState::Idle);
    }

    #[test]
    fn test_upload_flow() {
        let mut handler = MissionHandler::new(1, 1);

        // 1. Receive MISSION_COUNT (GCS sender: system_id=255, component_id=1)
        let count_msg = MISSION_COUNT_DATA {
            target_system: 1,
            target_component: 1,
            count: 2,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            opaque_id: 0,
        };
        let request = handler.handle_count(&count_msg, 0, 255, 1);
        assert_eq!(request.seq, 0);
        assert_eq!(request.target_system, 255); // Reply to GCS
        assert_eq!(request.target_component, 1);
        assert!(matches!(
            handler.state(),
            MissionState::UploadInProgress { count: 2, .. }
        ));

        // 2. Receive first MISSION_ITEM_INT
        let item1 = MISSION_ITEM_INT_DATA {
            target_system: 1,
            target_component: 1,
            seq: 0,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            current: 0,
            autocontinue: 1,
            param1: 0.0,
            param2: 5.0,
            param3: 0.0,
            param4: 0.0,
            x: 370000000,
            y: -1220000000,
            z: 100.0,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        };
        let result1 = handler.handle_item_int(&item1, 0);
        assert!(result1.is_ok());
        let request = result1.unwrap();
        assert!(request.is_some());
        assert_eq!(request.unwrap().seq, 1);
        assert_eq!(handler.storage().count(), 1);

        // 3. Receive second MISSION_ITEM_INT
        let item2 = MISSION_ITEM_INT_DATA {
            seq: 1,
            x: 370010000,
            y: -1220010000,
            z: 120.0,
            ..item1
        };
        let result2 = handler.handle_item_int(&item2, 0);
        assert!(result2.is_ok());
        assert!(result2.unwrap().is_none()); // Upload complete
        assert_eq!(handler.storage().count(), 2);
        assert_eq!(handler.state(), MissionState::Idle);
    }

    #[test]
    fn test_download_flow() {
        let mut handler = MissionHandler::new(1, 1);

        // Add waypoints to storage
        let wp1 = Waypoint::new(0, 370000000, -1220000000, 100.0);
        let wp2 = Waypoint::new(1, 370010000, -1220010000, 120.0);
        handler.storage_mut().add_waypoint(wp1).unwrap();
        handler.storage_mut().add_waypoint(wp2).unwrap();

        // 1. Receive MISSION_REQUEST_LIST (GCS sender: system_id=255, component_id=1)
        let request_list = MISSION_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        };
        let count_msg = handler.handle_request_list(&request_list, 0, 255, 1);
        assert_eq!(count_msg.count, 2);
        assert_eq!(count_msg.target_system, 255); // Reply to GCS
        assert_eq!(count_msg.target_component, 1);
        assert!(matches!(
            handler.state(),
            MissionState::DownloadInProgress { .. }
        ));

        // 2. Receive MISSION_REQUEST_INT for seq=0
        let request0 = MISSION_REQUEST_INT_DATA {
            target_system: 1,
            target_component: 1,
            seq: 0,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        };
        let result = handler.handle_request_int(&request0, 0);
        assert!(result.is_ok());
        let item = result.unwrap();
        assert_eq!(item.seq, 0);
        assert_eq!(item.x, 370000000);

        // 3. Receive MISSION_REQUEST_INT for seq=1
        let request1 = MISSION_REQUEST_INT_DATA { seq: 1, ..request0 };
        let result = handler.handle_request_int(&request1, 0);
        assert!(result.is_ok());
        let item = result.unwrap();
        assert_eq!(item.seq, 1);
        assert_eq!(item.x, 370010000);

        // 4. Receive MISSION_ACK
        let ack = MISSION_ACK_DATA {
            target_system: 1,
            target_component: 1,
            mavtype: MavMissionResult::MAV_MISSION_ACCEPTED,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION, // MAVLink v2 extension
            opaque_id: 0,                                           // MAVLink v2 extension
        };
        handler.handle_ack(&ack);
        assert_eq!(handler.state(), MissionState::Idle);
    }

    #[test]
    fn test_upload_sequence_error() {
        let mut handler = MissionHandler::new(1, 1);

        // Start upload (GCS sender: system_id=255, component_id=1)
        let count_msg = MISSION_COUNT_DATA {
            target_system: 1,
            target_component: 1,
            count: 2,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            opaque_id: 0,
        };
        handler.handle_count(&count_msg, 0, 255, 1);

        // Send waypoint with wrong sequence
        let item = MISSION_ITEM_INT_DATA {
            target_system: 1,
            target_component: 1,
            seq: 1, // Should be 0
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            current: 0,
            autocontinue: 1,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 0.0,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        };
        let result = handler.handle_item_int(&item, 0);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            MavMissionResult::MAV_MISSION_INVALID_SEQUENCE
        );
        assert_eq!(handler.state(), MissionState::Idle); // Aborted
    }

    #[test]
    fn test_timeout() {
        let mut handler = MissionHandler::new(1, 1);

        // Start upload (GCS sender: system_id=255, component_id=1)
        let count_msg = MISSION_COUNT_DATA {
            target_system: 1,
            target_component: 1,
            count: 2,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
            opaque_id: 0,
        };
        handler.handle_count(&count_msg, 0, 255, 1);

        // Check timeout after 6 seconds (> 5 second timeout)
        let timeout_result = handler.check_timeout(6_000_000);
        assert!(timeout_result.is_some());
        let (sys_id, comp_id) = timeout_result.unwrap();
        assert_eq!(sys_id, 255); // Original sender's IDs preserved
        assert_eq!(comp_id, 1);
        assert_eq!(handler.state(), MissionState::Idle);
    }

    #[test]
    fn test_download_waypoint_not_found() {
        let mut handler = MissionHandler::new(1, 1);

        // Start download with empty storage (GCS sender: system_id=255, component_id=1)
        let request_list = MISSION_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        };
        handler.handle_request_list(&request_list, 0, 255, 1);

        // Request nonexistent waypoint
        let request = MISSION_REQUEST_INT_DATA {
            target_system: 1,
            target_component: 1,
            seq: 0,
            mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION,
        };
        let result = handler.handle_request_int(&request, 0);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            MavMissionResult::MAV_MISSION_INVALID_SEQUENCE
        );
    }
}
