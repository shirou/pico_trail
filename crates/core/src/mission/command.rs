//! Mission Command Classification
//!
//! Helpers for classifying MAVLink mission commands as NAV (navigation)
//! or DO (immediate action) following ArduPilot's convention where
//! command IDs <= MAV_CMD_NAV_LAST (95) are NAV commands.

/// MAV_CMD_NAV_LAST: command IDs at or below this value are NAV commands.
///
/// Per MAVLink spec, NAV commands (0..=95) drive vehicle navigation.
/// Commands above this threshold are DO/condition commands.
pub const MAV_CMD_NAV_LAST: u16 = 95;

/// MAV_CMD_DO_CHANGE_SPEED command ID.
pub const MAV_CMD_DO_CHANGE_SPEED: u16 = 178;

/// Classify a command as NAV (drives navigation) or DO (immediate action).
///
/// NAV commands have IDs in the range 0..=95 (MAV_CMD_NAV_LAST).
/// All other commands are DO/condition commands.
pub fn is_nav_command(command_id: u16) -> bool {
    command_id <= MAV_CMD_NAV_LAST
}

/// Check if a command carries a geographic location (lat/lon).
///
/// Currently equivalent to `is_nav_command` — all NAV commands carry
/// location data while DO commands do not.
pub fn cmd_has_location(command_id: u16) -> bool {
    is_nav_command(command_id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_nav_command_nav_waypoint() {
        // MAV_CMD_NAV_WAYPOINT = 16
        assert!(is_nav_command(16));
    }

    #[test]
    fn test_is_nav_command_zero() {
        assert!(is_nav_command(0));
    }

    #[test]
    fn test_is_nav_command_at_boundary() {
        // MAV_CMD_NAV_LAST = 95 is still a NAV command
        assert!(is_nav_command(95));
    }

    #[test]
    fn test_is_nav_command_above_boundary() {
        // 96 is the first DO/condition command
        assert!(!is_nav_command(96));
    }

    #[test]
    fn test_is_nav_command_do_change_speed() {
        assert!(!is_nav_command(MAV_CMD_DO_CHANGE_SPEED));
    }

    #[test]
    fn test_cmd_has_location_nav() {
        assert!(cmd_has_location(16)); // NAV_WAYPOINT
        assert!(cmd_has_location(95)); // NAV_LAST
    }

    #[test]
    fn test_cmd_has_location_do() {
        assert!(!cmd_has_location(96));
        assert!(!cmd_has_location(178)); // DO_CHANGE_SPEED
    }
}
