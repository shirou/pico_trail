//! Mission Executor Trait and Event Types
//!
//! Defines the contract between the MissionSequencer and the vehicle mode
//! that physically executes mission commands. Modeled after ArduPilot's
//! AP_Mission 3-callback interface (cmd_start_fn, cmd_verify_fn,
//! mission_complete_fn).

use super::Waypoint;

/// Result of starting a mission command.
///
/// Returned by [`MissionExecutor::start_command`] to indicate how the
/// sequencer should handle the command slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandStartResult {
    /// Command accepted and executing (verify will be called each tick)
    Accepted,
    /// Command completed immediately (no verify needed)
    Complete,
    /// Command not recognized or not supported (skip)
    Unsupported,
}

/// Events emitted by the sequencer for telemetry coordination.
///
/// The firmware layer converts these into MAVLink messages:
/// - `CurrentChanged` -> MISSION_CURRENT
/// - `ItemReached` -> MISSION_ITEM_REACHED
/// - `MissionComplete` -> triggers `on_mission_complete` callback
/// - `MissionCleared` -> MISSION_ACK
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MissionEvent {
    /// Current target NAV command changed (seq index)
    CurrentChanged(u16),
    /// A mission item was reached after hold time (seq index)
    ItemReached(u16),
    /// Mission completed (no more NAV commands)
    MissionComplete,
    /// Mission cleared
    MissionCleared,
}

/// Command execution contract between sequencer and vehicle mode.
///
/// The sequencer calls these methods to delegate physical command execution
/// to the vehicle layer. Modeled after ArduPilot's AP_Mission 3-callback
/// interface:
/// - `cmd_start_fn` -> [`start_command`](MissionExecutor::start_command)
/// - `cmd_verify_fn` -> [`verify_command`](MissionExecutor::verify_command)
/// - `mission_complete_fn` -> [`on_mission_complete`](MissionExecutor::on_mission_complete)
pub trait MissionExecutor {
    /// Begin executing a mission command.
    ///
    /// For NAV commands: set navigation target, return `Accepted`.
    /// For DO commands: apply the effect (e.g., change speed), return `Complete`.
    /// For unknown commands: return `Unsupported` to skip.
    fn start_command(&mut self, cmd: &Waypoint) -> CommandStartResult;

    /// Check if a command has completed.
    ///
    /// Called each tick for the active NAV command slot.
    /// For NAV: return true when vehicle reached target (within acceptance radius).
    /// For DO: should not normally be called (most return `Complete` from start).
    fn verify_command(&mut self, cmd: &Waypoint) -> bool;

    /// Called when no more NAV commands remain in the mission.
    fn on_mission_complete(&mut self);
}
