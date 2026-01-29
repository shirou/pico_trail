//! Path Recorder for SmartRTL (firmware wrapper)
//!
//! Re-exports core path recorder types and provides the global static.

pub use pico_trail_core::navigation::path_recorder::{
    PathPoint, PathRecorder, ReturnPathIter, SRTL_POINTS_MAX,
};

use crate::core::traits::EmbassyState;

/// Global path recorder (protected by EmbassyState)
///
/// Access from arming handler to start/stop recording.
/// Access from SmartRTL mode to retrieve return path.
pub static PATH_RECORDER: EmbassyState<PathRecorder> = EmbassyState::new(PathRecorder::new());
