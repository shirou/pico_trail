//! RC Input Handler
//!
//! Handles MAVLink RC_CHANNELS messages from ground control stations.
//!
//! ## Messages Handled
//!
//! - `RC_CHANNELS` (ID 65): RC channel values from joystick/gamepad
//!
//! ## References
//!
//! - ADR-ea7fw-rc-input-processing: RC input design
//! - FR-993xy-rc-channels-processing: RC requirements
//! - MAVLink RC_CHANNELS: https://mavlink.io/en/messages/common.html#RC_CHANNELS

#[cfg(feature = "pico2_w")]
use crate::libraries::RC_INPUT;

/// Handle RC_CHANNELS message
///
/// Updates RC input state with normalized channel values.
///
/// # Arguments
///
/// * `raw_channels` - Raw channel values (0-65535) from RC_CHANNELS message
/// * `channel_count` - Number of active channels
/// * `current_time_us` - Current timestamp in microseconds
#[cfg(feature = "pico2_w")]
pub async fn handle_rc_channels(raw_channels: &[u16], channel_count: u8, current_time_us: u64) {
    // Lock RC input mutex
    let mut rc = RC_INPUT.lock().await;

    // Update RC state
    rc.update_from_mavlink(raw_channels, channel_count, current_time_us);

    // Log RC channel values at trace level
    #[cfg(feature = "defmt")]
    defmt::trace!(
        "RC_CHANNELS: ch1={}, ch3={}, count={}",
        rc.get_channel(1),
        rc.get_channel(3),
        channel_count
    );
}
