//! RC Input Handler
//!
//! Handles MAVLink RC input messages from ground control stations.
//!
//! ## Messages Handled
//!
//! - `RC_CHANNELS` (ID 65): RC channel values from receiver
//! - `RC_CHANNELS_OVERRIDE` (ID 70): RC channel override from GCS joystick/gamepad

use crate::traits::sync::SharedState;

#[cfg(feature = "embassy")]
use crate::rc::RC_INPUT;

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
}

impl Default for RcInputHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl RcInputHandler {
    /// Handle RC_CHANNELS message
    ///
    /// Updates RC input state with normalized channel values.
    #[cfg(feature = "embassy")]
    pub async fn handle_rc_channels(
        &mut self,
        rc_data: &mavlink::common::RC_CHANNELS_DATA,
        current_time_us: u64,
    ) {
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

        RC_INPUT.with_mut(|rc| {
            rc.update_from_mavlink(&channels, rc_data.chancount, current_time_us);
        });
    }

    /// Handle RC_CHANNELS_OVERRIDE message
    ///
    /// Updates RC input state with override channel values from GCS joystick/gamepad.
    #[cfg(feature = "embassy")]
    pub async fn handle_rc_channels_override(
        &mut self,
        rc_override: &mavlink::common::RC_CHANNELS_OVERRIDE_DATA,
        current_time_us: u64,
    ) -> bool {
        if rc_override.target_system != 1 && rc_override.target_system != 0 {
            return false;
        }

        let channels = [
            rc_override.chan1_raw,
            rc_override.chan2_raw,
            rc_override.chan3_raw,
            rc_override.chan4_raw,
            rc_override.chan5_raw,
            rc_override.chan6_raw,
            rc_override.chan7_raw,
            rc_override.chan8_raw,
            0,
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

        RC_INPUT.with_mut(|rc| {
            rc.update_from_pwm(&channels, 8, current_time_us);
        });

        true
    }
}
