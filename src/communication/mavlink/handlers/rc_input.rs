//! RC Input Handler
//!
//! Handles MAVLink RC input messages from ground control stations.
//!
//! ## Messages Handled
//!
//! - `RC_CHANNELS` (ID 65): RC channel values from receiver
//! - `RC_CHANNELS_OVERRIDE` (ID 70): RC channel override from GCS joystick/gamepad
//!
//! ## References
//!
//! - ADR-ea7fw-rc-input-processing: RC input design
//! - FR-993xy-rc-channels-processing: RC requirements
//! - MAVLink RC_CHANNELS: https://mavlink.io/en/messages/common.html#RC_CHANNELS
//! - MAVLink RC_CHANNELS_OVERRIDE: https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE

#[cfg(feature = "embassy")]
use crate::libraries::RC_INPUT;

/// RC input message handler
///
/// Processes RC_CHANNELS and RC_CHANNELS_OVERRIDE messages to update the global RC state.
/// This handler is stateless and updates the global RC_INPUT directly.
pub struct RcInputHandler;

impl RcInputHandler {
    /// Create a new RC input handler
    pub fn new() -> Self {
        Self
    }

    /// Handle RC_CHANNELS message
    ///
    /// Updates RC input state with normalized channel values.
    ///
    /// # Arguments
    ///
    /// * `rc_data` - RC_CHANNELS message data
    /// * `current_time_us` - Current timestamp in microseconds
    #[cfg(feature = "embassy")]
    pub async fn handle_rc_channels(
        &mut self,
        rc_data: &mavlink::common::RC_CHANNELS_DATA,
        current_time_us: u64,
    ) {
        // Extract channel values into array
        let channels = [
            rc_data.chan1_raw,
            rc_data.chan2_raw,
            rc_data.chan3_raw,
            rc_data.chan4_raw,
            rc_data.chan5_raw,
            rc_data.chan6_raw,
            rc_data.chan7_raw,
            rc_data.chan8_raw,
            rc_data.chan9_raw,
            rc_data.chan10_raw,
            rc_data.chan11_raw,
            rc_data.chan12_raw,
            rc_data.chan13_raw,
            rc_data.chan14_raw,
            rc_data.chan15_raw,
            rc_data.chan16_raw,
            rc_data.chan17_raw,
            rc_data.chan18_raw,
        ];

        // Lock RC input mutex
        let mut rc = RC_INPUT.lock().await;

        // Update RC state
        rc.update_from_mavlink(&channels, rc_data.chancount, current_time_us);
    }

    /// Handle RC_CHANNELS_OVERRIDE message
    ///
    /// Updates RC input state with override channel values from GCS joystick/gamepad.
    /// This is typically sent by Mission Planner or QGroundControl when using a joystick.
    ///
    /// RC_CHANNELS_OVERRIDE uses PWM format (1000-2000), not the 0-65535 format used by RC_CHANNELS.
    ///
    /// # Arguments
    ///
    /// * `rc_override` - RC_CHANNELS_OVERRIDE message data
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// `true` if the message was processed (target system matches), `false` otherwise
    #[cfg(feature = "embassy")]
    pub async fn handle_rc_channels_override(
        &mut self,
        rc_override: &mavlink::common::RC_CHANNELS_OVERRIDE_DATA,
        current_time_us: u64,
    ) -> bool {
        // Only process messages for our system (ID 1) or broadcast (ID 0)
        if rc_override.target_system != 1 && rc_override.target_system != 0 {
            return false;
        }

        // Extract channel values into array
        // RC_CHANNELS_OVERRIDE has 8 channels (chan1-8)
        let channels = [
            rc_override.chan1_raw,
            rc_override.chan2_raw,
            rc_override.chan3_raw,
            rc_override.chan4_raw,
            rc_override.chan5_raw,
            rc_override.chan6_raw,
            rc_override.chan7_raw,
            rc_override.chan8_raw,
            0, // chan9-18 not available in RC_CHANNELS_OVERRIDE
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];

        // Lock RC input mutex
        let mut rc = RC_INPUT.lock().await;

        // Update RC state using PWM normalization (1000-2000 range)
        rc.update_from_pwm(&channels, 8, current_time_us);

        true
    }

    /// No-op handler for non-embassy builds
    #[cfg(not(feature = "embassy"))]
    pub async fn handle_rc_channels(
        &mut self,
        _rc_data: &mavlink::common::RC_CHANNELS_DATA,
        _current_time_us: u64,
    ) {
        // No-op on non-embedded platforms
    }

    /// No-op handler for non-embassy builds
    #[cfg(not(feature = "embassy"))]
    pub async fn handle_rc_channels_override(
        &mut self,
        _rc_override: &mavlink::common::RC_CHANNELS_OVERRIDE_DATA,
        _current_time_us: u64,
    ) -> bool {
        // No-op on non-embedded platforms
        false
    }
}

impl Default for RcInputHandler {
    fn default() -> Self {
        Self::new()
    }
}
