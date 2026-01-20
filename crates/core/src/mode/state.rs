//! Mode state types
//!
//! Pure state types for control modes. These types store mode-specific
//! state that persists across update cycles.

/// Auto mode internal state
#[derive(Debug, Clone, Copy, Default)]
pub struct AutoState {
    /// Flag indicating if navigation is active
    pub navigation_active: bool,
    /// Flag indicating if mission is complete
    pub mission_complete: bool,
    /// Current waypoint index (for logging)
    pub current_wp_index: u16,
}

/// Guided mode internal state
#[derive(Debug, Clone, Copy, Default)]
pub struct GuidedState {
    /// Flag indicating if navigation is active
    pub navigation_active: bool,
    /// Flag indicating if target has been reached
    pub at_target: bool,
}

/// RTL mode internal state
#[derive(Debug, Clone, Copy, Default)]
pub struct RtlState {
    /// Home position target latitude
    pub home_lat: f32,
    /// Home position target longitude
    pub home_lon: f32,
    /// Arrival flag
    pub arrived: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auto_state_default() {
        let state = AutoState::default();
        assert!(!state.navigation_active);
        assert!(!state.mission_complete);
        assert_eq!(state.current_wp_index, 0);
    }

    #[test]
    fn test_guided_state_default() {
        let state = GuidedState::default();
        assert!(!state.navigation_active);
        assert!(!state.at_target);
    }

    #[test]
    fn test_rtl_state_default() {
        let state = RtlState::default();
        assert!((state.home_lat - 0.0).abs() < 0.001);
        assert!((state.home_lon - 0.0).abs() < 0.001);
        assert!(!state.arrived);
    }
}
