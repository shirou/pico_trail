//! Mission Sequencer
//!
//! Platform-agnostic state machine that owns mission command classification,
//! dual-slot NAV/DO advancement, hold time management, and mission speed
//! override tracking. Modeled after ArduPilot's AP_Mission dual-slot pattern.
//!
//! The sequencer does not know about MAVLink, actuators, GPS, or any platform
//! service. It drives physical execution through the [`MissionExecutor`] trait.

use heapless::Vec;

use super::command::{is_nav_command, MAV_CMD_DO_CHANGE_SPEED};
use super::executor::{CommandStartResult, MissionEvent, MissionExecutor};
use super::state::MissionState;
use super::MissionStorage;

/// Maximum mission events emitted per update cycle.
pub const MAX_MISSION_EVENTS: usize = 4;

/// Internal flags for sequencer state tracking.
#[derive(Clone, Copy, Debug, Default)]
struct SequencerFlags {
    /// NAV command slot has an active command
    nav_cmd_loaded: bool,
    /// DO command slot has an active command
    do_cmd_loaded: bool,
    /// All DO commands between current and next NAV are processed
    do_cmd_all_done: bool,
}

/// Mission sequencer — drives mission execution through MissionExecutor.
///
/// Platform-agnostic state machine that owns:
/// - Command classification (NAV vs DO)
/// - Dual-slot NAV/DO advancement (ArduPilot pattern)
/// - Hold time management (NAV_WAYPOINT param1)
/// - Mission speed override tracking (DO_CHANGE_SPEED)
///
/// Does not know about MAVLink, actuators, GPS, or any platform service.
pub struct MissionSequencer {
    state: MissionState,
    flags: SequencerFlags,
    /// Index of the active NAV command in MissionStorage
    nav_cmd_index: u16,
    /// Index of the active DO command in MissionStorage
    do_cmd_index: u16,
    /// Timestamp (ms) when hold started at current waypoint
    holding_since: Option<u64>,
    /// Speed override from DO_CHANGE_SPEED (None = use default)
    mission_speed: Option<f32>,
    /// Flag indicating the firmware should re-load the current NAV target
    /// (set by `set_current()`, cleared by `clear_nav_refresh()`)
    nav_refresh_needed: bool,
}

impl MissionSequencer {
    /// Create a new sequencer in Idle state.
    /// Create a new sequencer in Idle state (const fn for static initialization).
    pub const fn new() -> Self {
        Self {
            state: MissionState::Idle,
            flags: SequencerFlags {
                nav_cmd_loaded: false,
                do_cmd_loaded: false,
                do_cmd_all_done: false,
            },
            nav_cmd_index: 0,
            do_cmd_index: 0,
            holding_since: None,
            mission_speed: None,
            nav_refresh_needed: false,
        }
    }

    /// Get current mission state.
    pub fn state(&self) -> MissionState {
        self.state
    }

    /// Get index of the active NAV command (for MISSION_CURRENT).
    pub fn current_nav_index(&self) -> u16 {
        self.nav_cmd_index
    }

    /// Get speed override from DO_CHANGE_SPEED (None = use default).
    pub fn mission_speed(&self) -> Option<f32> {
        self.mission_speed
    }

    /// Check if the sequencer is currently holding at a waypoint.
    ///
    /// Returns `true` when the vehicle has reached the target waypoint and
    /// is waiting for the hold timer (param1) to expire before advancing.
    pub fn is_holding(&self) -> bool {
        self.holding_since.is_some()
    }

    /// Check if the firmware needs to re-load the current NAV target.
    ///
    /// Set to `true` after `set_current()` so that the firmware layer can
    /// re-issue `start_command` for the new waypoint on its next update tick.
    pub fn needs_nav_refresh(&self) -> bool {
        self.nav_refresh_needed
    }

    /// Clear the navigation refresh flag after the firmware has re-loaded
    /// the NAV target.
    pub fn clear_nav_refresh(&mut self) {
        self.nav_refresh_needed = false;
    }

    /// Start mission execution from index 0.
    ///
    /// Scans forward to load the first NAV command and any preceding DO commands.
    /// Returns an empty event list if the mission is empty (state stays Idle).
    pub fn start(
        &mut self,
        storage: &MissionStorage,
        executor: &mut dyn MissionExecutor,
    ) -> Vec<MissionEvent, MAX_MISSION_EVENTS> {
        let mut events = Vec::new();

        if storage.is_empty() {
            return events;
        }

        self.state = MissionState::Running;
        self.flags = SequencerFlags::default();
        self.holding_since = None;
        self.mission_speed = None;
        self.nav_cmd_index = 0;
        self.do_cmd_index = 0;

        // Scan from index 0 to find the first NAV command
        if self.load_first_nav(storage, 0, executor, &mut events) {
            // Load any DO commands before the NAV
            self.load_do_commands_before_nav(storage, executor);
        } else {
            // No NAV commands at all
            self.state = MissionState::Idle;
        }

        events
    }

    /// Stop mission execution, preserving indices for potential resume.
    pub fn stop(&mut self) -> Vec<MissionEvent, MAX_MISSION_EVENTS> {
        self.state = MissionState::Idle;
        self.flags.nav_cmd_loaded = false;
        self.flags.do_cmd_loaded = false;
        self.holding_since = None;
        self.mission_speed = None;
        self.nav_refresh_needed = false;
        Vec::new()
    }

    /// Main tick: drive NAV/DO slots and emit events.
    ///
    /// Called each update cycle (e.g. 50Hz) by the vehicle mode.
    /// Processes DO slot first, then NAV slot.
    pub fn update(
        &mut self,
        storage: &MissionStorage,
        executor: &mut dyn MissionExecutor,
        now_ms: u64,
    ) -> Vec<MissionEvent, MAX_MISSION_EVENTS> {
        let mut events = Vec::new();

        if self.state != MissionState::Running {
            return events;
        }

        // Process DO slot
        self.process_do_slot(storage, executor);

        // Process NAV slot
        self.process_nav_slot(storage, executor, now_ms, &mut events);

        events
    }

    /// Handle MISSION_SET_CURRENT from GCS.
    ///
    /// Resets slots and scans from the given index. Returns error if
    /// the index is out of bounds.
    pub fn set_current(
        &mut self,
        index: u16,
        storage: &MissionStorage,
        executor: &mut dyn MissionExecutor,
    ) -> Result<Vec<MissionEvent, MAX_MISSION_EVENTS>, &'static str> {
        if index >= storage.count() {
            return Err("Index out of bounds");
        }

        let mut events = Vec::new();

        self.flags = SequencerFlags::default();
        self.holding_since = None;

        if self.state != MissionState::Running {
            self.state = MissionState::Running;
        }

        if self.load_first_nav(storage, index, executor, &mut events) {
            self.load_do_commands_before_nav(storage, executor);
        }

        self.nav_refresh_needed = true;

        Ok(events)
    }

    /// Handle MISSION_CLEAR_ALL from GCS.
    ///
    /// Clears storage and resets sequencer to Idle.
    pub fn clear(&mut self, storage: &mut MissionStorage) -> Vec<MissionEvent, MAX_MISSION_EVENTS> {
        let mut events = Vec::new();

        storage.clear();
        self.state = MissionState::Idle;
        self.flags = SequencerFlags::default();
        self.nav_cmd_index = 0;
        self.do_cmd_index = 0;
        self.holding_since = None;
        self.mission_speed = None;
        self.nav_refresh_needed = false;

        let _ = events.push(MissionEvent::MissionCleared);
        events
    }

    // ========================================================================
    // Internal methods
    // ========================================================================

    /// Scan forward from `start_index` to find and load the first NAV command.
    /// Returns true if a NAV command was found.
    fn load_first_nav(
        &mut self,
        storage: &MissionStorage,
        start_index: u16,
        executor: &mut dyn MissionExecutor,
        events: &mut Vec<MissionEvent, MAX_MISSION_EVENTS>,
    ) -> bool {
        for i in start_index..storage.count() {
            if let Some(wp) = storage.get_waypoint(i) {
                if is_nav_command(wp.command) {
                    self.nav_cmd_index = i;
                    self.flags.nav_cmd_loaded = true;
                    executor.start_command(wp);
                    let _ = events.push(MissionEvent::CurrentChanged(i));
                    return true;
                }
            }
        }
        false
    }

    /// Load DO commands that appear between the start of the scan region
    /// and the current NAV command. Called after loading a NAV command.
    fn load_do_commands_before_nav(
        &mut self,
        storage: &MissionStorage,
        executor: &mut dyn MissionExecutor,
    ) {
        // DO commands after the current NAV, up to the next NAV boundary
        let scan_start = self.nav_cmd_index + 1;
        self.do_cmd_index = scan_start;
        self.flags.do_cmd_loaded = false;
        self.flags.do_cmd_all_done = false;

        self.advance_do_cmd(storage, executor);
    }

    /// Advance the DO command slot to the next DO command.
    /// Stops scanning at a NAV command boundary.
    fn advance_do_cmd(&mut self, storage: &MissionStorage, executor: &mut dyn MissionExecutor) {
        let start = if self.flags.do_cmd_loaded {
            self.do_cmd_index + 1
        } else {
            self.do_cmd_index
        };

        self.flags.do_cmd_loaded = false;

        for i in start..storage.count() {
            if let Some(wp) = storage.get_waypoint(i) {
                if is_nav_command(wp.command) {
                    // Hit the next NAV boundary — all DOs done
                    self.flags.do_cmd_all_done = true;
                    return;
                }

                // This is a DO command
                self.do_cmd_index = i;

                // Handle DO_CHANGE_SPEED: update mission_speed with validation
                if wp.command == MAV_CMD_DO_CHANGE_SPEED {
                    let speed = wp.param2;
                    if speed > 0.0 && !speed.is_nan() && !speed.is_infinite() {
                        self.mission_speed = Some(speed);
                    }
                    // Invalid values (<=0, NaN, infinite) are silently rejected
                }

                let result = executor.start_command(wp);
                match result {
                    CommandStartResult::Complete | CommandStartResult::Unsupported => {
                        // Command completed immediately or unsupported — continue scanning
                        continue;
                    }
                    CommandStartResult::Accepted => {
                        // Command needs verification each tick
                        self.flags.do_cmd_loaded = true;
                        return;
                    }
                }
            }
        }

        // Reached end of storage without hitting another NAV
        self.flags.do_cmd_all_done = true;
    }

    /// Advance the NAV command slot to the next NAV command after the current one.
    fn advance_nav_cmd(
        &mut self,
        storage: &MissionStorage,
        executor: &mut dyn MissionExecutor,
        events: &mut Vec<MissionEvent, MAX_MISSION_EVENTS>,
    ) {
        self.flags.nav_cmd_loaded = false;
        self.holding_since = None;

        let next_start = self.nav_cmd_index + 1;

        if self.load_first_nav(storage, next_start, executor, events) {
            // Found next NAV — load DO commands between this NAV and the next
            self.load_do_commands_before_nav(storage, executor);
        } else {
            // No more NAV commands — mission complete
            self.state = MissionState::Completed;
            executor.on_mission_complete();
            let _ = events.push(MissionEvent::MissionComplete);
        }
    }

    /// Process the DO command slot during update.
    fn process_do_slot(&mut self, storage: &MissionStorage, executor: &mut dyn MissionExecutor) {
        if self.flags.do_cmd_all_done {
            return;
        }

        if self.flags.do_cmd_loaded {
            // Verify the active DO command
            if let Some(wp) = storage.get_waypoint(self.do_cmd_index) {
                if executor.verify_command(wp) {
                    // DO command completed — advance to next
                    self.advance_do_cmd(storage, executor);
                }
            }
        } else {
            // No DO loaded but not all done — try to load next
            self.advance_do_cmd(storage, executor);
        }
    }

    /// Process the NAV command slot during update.
    fn process_nav_slot(
        &mut self,
        storage: &MissionStorage,
        executor: &mut dyn MissionExecutor,
        now_ms: u64,
        events: &mut Vec<MissionEvent, MAX_MISSION_EVENTS>,
    ) {
        if !self.flags.nav_cmd_loaded {
            return;
        }

        let wp = match storage.get_waypoint(self.nav_cmd_index) {
            Some(wp) => wp,
            None => return,
        };

        if !executor.verify_command(wp) {
            // Not at target yet
            return;
        }

        // At target — check hold time
        let hold_seconds = wp.param1;
        if hold_seconds > 0.0 {
            match self.holding_since {
                None => {
                    // Start holding
                    self.holding_since = Some(now_ms);
                    return;
                }
                Some(start) => {
                    let elapsed_ms = now_ms.saturating_sub(start);
                    let hold_ms = (hold_seconds * 1000.0) as u64;
                    if elapsed_ms < hold_ms {
                        // Still holding
                        return;
                    }
                    // Hold complete — fall through to advance
                }
            }
        }

        // Emit ItemReached and advance to next NAV
        let _ = events.push(MissionEvent::ItemReached(self.nav_cmd_index));
        self.advance_nav_cmd(storage, executor, events);
    }
}

impl Default for MissionSequencer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::Waypoint;

    // ========================================================================
    // MockExecutor
    // ========================================================================

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum MockCall {
        Start(u16),  // command seq
        Verify(u16), // command seq
        MissionComplete,
    }

    struct MockExecutor {
        calls: Vec<MockCall, 64>,
        /// When verify is called for a command at this seq, return true
        verify_true_for: Vec<u16, 16>,
        /// Default start result
        start_result: CommandStartResult,
    }

    impl MockExecutor {
        fn new() -> Self {
            Self {
                calls: Vec::new(),
                verify_true_for: Vec::new(),
                start_result: CommandStartResult::Accepted,
            }
        }

        fn set_verify_true(&mut self, seq: u16) {
            let _ = self.verify_true_for.push(seq);
        }
    }

    impl MissionExecutor for MockExecutor {
        fn start_command(&mut self, cmd: &Waypoint) -> CommandStartResult {
            let _ = self.calls.push(MockCall::Start(cmd.seq));
            if is_nav_command(cmd.command) {
                self.start_result
            } else {
                // DO commands complete immediately by default
                CommandStartResult::Complete
            }
        }

        fn verify_command(&mut self, cmd: &Waypoint) -> bool {
            let _ = self.calls.push(MockCall::Verify(cmd.seq));
            self.verify_true_for.contains(&cmd.seq)
        }

        fn on_mission_complete(&mut self) {
            let _ = self.calls.push(MockCall::MissionComplete);
        }
    }

    // ========================================================================
    // Helpers
    // ========================================================================

    fn nav_wp(seq: u16) -> Waypoint {
        Waypoint {
            seq,
            command: 16, // NAV_WAYPOINT
            autocontinue: 1,
            ..Waypoint::default()
        }
    }

    fn nav_wp_with_hold(seq: u16, hold_seconds: f32) -> Waypoint {
        Waypoint {
            seq,
            command: 16, // NAV_WAYPOINT
            autocontinue: 1,
            param1: hold_seconds,
            ..Waypoint::default()
        }
    }

    fn do_change_speed(seq: u16, speed: f32) -> Waypoint {
        Waypoint {
            seq,
            command: MAV_CMD_DO_CHANGE_SPEED,
            autocontinue: 1,
            param2: speed,
            ..Waypoint::default()
        }
    }

    fn do_generic(seq: u16, command: u16) -> Waypoint {
        Waypoint {
            seq,
            command,
            autocontinue: 1,
            ..Waypoint::default()
        }
    }

    // ========================================================================
    // Tests: Start/Stop lifecycle
    // ========================================================================

    #[test]
    fn test_start_stop_lifecycle() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        assert_eq!(seq.state(), MissionState::Idle);

        let events = seq.start(&storage, &mut exec);
        assert_eq!(seq.state(), MissionState::Running);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0], MissionEvent::CurrentChanged(0));

        seq.stop();
        assert_eq!(seq.state(), MissionState::Idle);
    }

    #[test]
    fn test_start_empty_mission() {
        let storage = MissionStorage::new();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        let events = seq.start(&storage, &mut exec);
        assert_eq!(seq.state(), MissionState::Idle);
        assert!(events.is_empty());
    }

    // ========================================================================
    // Tests: Single NAV waypoint
    // ========================================================================

    #[test]
    fn test_single_nav_waypoint_mission() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        // Not at target yet
        let events = seq.update(&storage, &mut exec, 0);
        assert!(events.is_empty());
        assert_eq!(seq.state(), MissionState::Running);

        // Arrive at target
        exec.set_verify_true(0);
        let events = seq.update(&storage, &mut exec, 1000);
        assert!(events.contains(&MissionEvent::ItemReached(0)));
        assert!(events.contains(&MissionEvent::MissionComplete));
        assert_eq!(seq.state(), MissionState::Completed);
    }

    // ========================================================================
    // Tests: Multi-NAV waypoint advancement
    // ========================================================================

    #[test]
    fn test_multi_nav_waypoint_advancement() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.current_nav_index(), 0);

        // Arrive at WP 0
        exec.set_verify_true(0);
        let events = seq.update(&storage, &mut exec, 1000);
        assert!(events.contains(&MissionEvent::ItemReached(0)));
        assert!(events.contains(&MissionEvent::CurrentChanged(1)));
        assert_eq!(seq.current_nav_index(), 1);

        // Arrive at WP 1
        exec.set_verify_true(1);
        let events = seq.update(&storage, &mut exec, 2000);
        assert!(events.contains(&MissionEvent::ItemReached(1)));
        assert!(events.contains(&MissionEvent::CurrentChanged(2)));
        assert_eq!(seq.current_nav_index(), 2);

        // Arrive at WP 2 (last)
        exec.set_verify_true(2);
        let events = seq.update(&storage, &mut exec, 3000);
        assert!(events.contains(&MissionEvent::ItemReached(2)));
        assert!(events.contains(&MissionEvent::MissionComplete));
        assert_eq!(seq.state(), MissionState::Completed);
    }

    // ========================================================================
    // Tests: DO commands execute between NAVs
    // ========================================================================

    #[test]
    fn test_do_commands_execute_between_navs() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, 2.0)).unwrap();
        storage.add_waypoint(do_generic(2, 200)).unwrap(); // some other DO
        storage.add_waypoint(nav_wp(3)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        // DO commands at seq 1 and 2 should have been started during load
        assert!(exec.calls.contains(&MockCall::Start(1)));
        assert!(exec.calls.contains(&MockCall::Start(2)));
    }

    #[test]
    fn test_do_commands_stop_at_nav_boundary() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, 2.0)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        storage.add_waypoint(do_generic(3, 200)).unwrap(); // after second NAV
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        // Only DO at seq 1 should be started (seq 3 is after NAV at seq 2)
        let start_calls: Vec<u16, 16> = exec
            .calls
            .iter()
            .filter_map(|c| {
                if let MockCall::Start(s) = c {
                    Some(*s)
                } else {
                    None
                }
            })
            .collect();
        assert!(start_calls.contains(&0)); // NAV
        assert!(start_calls.contains(&1)); // DO
        assert!(!start_calls.contains(&3)); // not yet
    }

    // ========================================================================
    // Tests: Mixed NAV/DO mission end-to-end
    // ========================================================================

    #[test]
    fn test_mixed_nav_do_mission() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, 3.0)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        assert_eq!(seq.current_nav_index(), 0);
        assert_eq!(seq.mission_speed(), Some(3.0));

        // Arrive at WP 0
        exec.set_verify_true(0);
        let events = seq.update(&storage, &mut exec, 1000);
        assert!(events.contains(&MissionEvent::ItemReached(0)));
        assert!(events.contains(&MissionEvent::CurrentChanged(2)));

        // Arrive at WP 2
        exec.set_verify_true(2);
        let events = seq.update(&storage, &mut exec, 2000);
        assert!(events.contains(&MissionEvent::ItemReached(2)));
        assert!(events.contains(&MissionEvent::MissionComplete));
    }

    // ========================================================================
    // Tests: Hold time
    // ========================================================================

    #[test]
    fn test_hold_time_zero_advances_immediately() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp_with_hold(0, 0.0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        exec.set_verify_true(0);
        let events = seq.update(&storage, &mut exec, 1000);
        // Should advance immediately (no hold)
        assert!(events.contains(&MissionEvent::ItemReached(0)));
        assert!(events.contains(&MissionEvent::CurrentChanged(1)));
    }

    #[test]
    fn test_hold_time_waits_before_advance() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp_with_hold(0, 5.0)).unwrap(); // 5 second hold
        storage.add_waypoint(nav_wp(1)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        exec.set_verify_true(0);

        // First tick at target — starts hold, does not advance
        let events = seq.update(&storage, &mut exec, 1000);
        assert!(events.is_empty());

        // 2 seconds later — still holding
        let events = seq.update(&storage, &mut exec, 3000);
        assert!(events.is_empty());

        // 5 seconds after hold started — should advance
        let events = seq.update(&storage, &mut exec, 6000);
        assert!(events.contains(&MissionEvent::ItemReached(0)));
        assert!(events.contains(&MissionEvent::CurrentChanged(1)));
    }

    #[test]
    fn test_item_reached_after_hold_complete() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp_with_hold(0, 2.0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        exec.set_verify_true(0);

        // Start hold
        seq.update(&storage, &mut exec, 0);
        // Hold not complete yet
        let events = seq.update(&storage, &mut exec, 1000);
        assert!(events.is_empty());

        // Hold complete
        let events = seq.update(&storage, &mut exec, 2000);
        assert!(events.contains(&MissionEvent::ItemReached(0)));
    }

    // ========================================================================
    // Tests: DO_CHANGE_SPEED
    // ========================================================================

    #[test]
    fn test_do_change_speed_updates_mission_speed() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, 5.5)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        assert_eq!(seq.mission_speed(), None);

        seq.start(&storage, &mut exec);
        assert_eq!(seq.mission_speed(), Some(5.5));
    }

    #[test]
    fn test_mission_speed_none_by_default() {
        let seq = MissionSequencer::new();
        assert_eq!(seq.mission_speed(), None);
    }

    #[test]
    fn test_mission_speed_reset_on_stop() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, 3.0)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.mission_speed(), Some(3.0));

        seq.stop();
        assert_eq!(seq.mission_speed(), None);
    }

    // ========================================================================
    // Tests: set_current
    // ========================================================================

    #[test]
    fn test_set_current_reloads_nav_slot() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.current_nav_index(), 0);

        let events = seq.set_current(2, &storage, &mut exec).unwrap();
        assert_eq!(seq.current_nav_index(), 2);
        assert!(events.contains(&MissionEvent::CurrentChanged(2)));
    }

    #[test]
    fn test_set_current_invalid_index() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        let result = seq.set_current(5, &storage, &mut exec);
        assert!(result.is_err());
    }

    // ========================================================================
    // Tests: clear
    // ========================================================================

    #[test]
    fn test_clear_resets_state() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.state(), MissionState::Running);

        let events = seq.clear(&mut storage);
        assert_eq!(seq.state(), MissionState::Idle);
        assert!(storage.is_empty());
        assert!(events.contains(&MissionEvent::MissionCleared));
        assert_eq!(seq.current_nav_index(), 0);
        assert_eq!(seq.mission_speed(), None);
    }

    // ========================================================================
    // Tests: Events
    // ========================================================================

    #[test]
    fn test_current_changed_on_nav_advance() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        let events = seq.start(&storage, &mut exec);
        assert!(events.contains(&MissionEvent::CurrentChanged(0)));

        exec.set_verify_true(0);
        let events = seq.update(&storage, &mut exec, 1000);
        assert!(events.contains(&MissionEvent::CurrentChanged(1)));
    }

    #[test]
    fn test_mission_complete_event() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        exec.set_verify_true(0);
        let events = seq.update(&storage, &mut exec, 1000);
        assert!(events.contains(&MissionEvent::MissionComplete));
        assert!(exec.calls.contains(&MockCall::MissionComplete));
    }

    // ========================================================================
    // Tests: Update while not running
    // ========================================================================

    #[test]
    fn test_update_while_idle_returns_empty() {
        let storage = MissionStorage::new();
        let mut exec = MockExecutor::new();
        let mut seq = MissionSequencer::new();

        let events = seq.update(&storage, &mut exec, 1000);
        assert!(events.is_empty());
    }

    #[test]
    fn test_update_after_complete_returns_empty() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        exec.set_verify_true(0);
        seq.update(&storage, &mut exec, 1000);
        assert_eq!(seq.state(), MissionState::Completed);

        // Further updates should be no-ops
        exec.calls.clear();
        let events = seq.update(&storage, &mut exec, 2000);
        assert!(events.is_empty());
    }

    // ========================================================================
    // Tests: is_holding() observability (Phase 5)
    // ========================================================================

    #[test]
    fn test_is_holding_lifecycle() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp_with_hold(0, 5.0)).unwrap(); // 5s hold
        storage.add_waypoint(nav_wp(1)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        // Not at target yet — not holding
        assert!(!seq.is_holding());

        // Arrive at target — hold starts
        exec.set_verify_true(0);
        seq.update(&storage, &mut exec, 1000);
        assert!(seq.is_holding());

        // Still holding
        seq.update(&storage, &mut exec, 3000);
        assert!(seq.is_holding());

        // Hold completes — advances to next NAV
        seq.update(&storage, &mut exec, 6000);
        assert!(!seq.is_holding());
    }

    // ========================================================================
    // Tests: nav_refresh_needed flag (Phase 5)
    // ========================================================================

    #[test]
    fn test_nav_refresh_flag_after_set_current() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);

        // Initially no refresh needed
        assert!(!seq.needs_nav_refresh());

        // set_current sets the flag
        seq.set_current(2, &storage, &mut exec).unwrap();
        assert!(seq.needs_nav_refresh());

        // clear_nav_refresh clears it
        seq.clear_nav_refresh();
        assert!(!seq.needs_nav_refresh());
    }

    #[test]
    fn test_nav_refresh_cleared_on_stop() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        seq.set_current(1, &storage, &mut exec).unwrap();
        assert!(seq.needs_nav_refresh());

        seq.stop();
        assert!(!seq.needs_nav_refresh());
    }

    #[test]
    fn test_nav_refresh_cleared_on_clear() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(nav_wp(1)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        seq.set_current(1, &storage, &mut exec).unwrap();
        assert!(seq.needs_nav_refresh());

        seq.clear(&mut storage);
        assert!(!seq.needs_nav_refresh());
    }

    // ========================================================================
    // Tests: Speed validation (Phase 5)
    // ========================================================================

    #[test]
    fn test_speed_validation_valid_value() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, 3.5)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.mission_speed(), Some(3.5));
    }

    #[test]
    fn test_speed_validation_zero_rejected() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, 0.0)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.mission_speed(), None);
    }

    #[test]
    fn test_speed_validation_negative_rejected() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, -1.0)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.mission_speed(), None);
    }

    #[test]
    fn test_speed_validation_nan_rejected() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, f32::NAN)).unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.mission_speed(), None);
    }

    #[test]
    fn test_speed_validation_infinite_rejected() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage
            .add_waypoint(do_change_speed(1, f32::INFINITY))
            .unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.mission_speed(), None);
    }

    #[test]
    fn test_speed_validation_neg_infinite_rejected() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage
            .add_waypoint(do_change_speed(1, f32::NEG_INFINITY))
            .unwrap();
        storage.add_waypoint(nav_wp(2)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        assert_eq!(seq.mission_speed(), None);
    }

    #[test]
    fn test_speed_validation_preserves_previous_on_invalid() {
        let mut storage = MissionStorage::new();
        storage.add_waypoint(nav_wp(0)).unwrap();
        storage.add_waypoint(do_change_speed(1, 5.0)).unwrap(); // valid
        storage.add_waypoint(do_change_speed(2, -1.0)).unwrap(); // invalid
        storage.add_waypoint(nav_wp(3)).unwrap();
        let mut exec = MockExecutor::new();

        let mut seq = MissionSequencer::new();
        seq.start(&storage, &mut exec);
        // Valid speed stored, invalid rejected — previous value preserved
        assert_eq!(seq.mission_speed(), Some(5.0));
    }
}
