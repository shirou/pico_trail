//! Heading exponential moving average (EMA) filter
//!
//! Provides angle-aware smoothing to reduce vibration-induced noise
//! in AHRS heading readings before they reach the navigation controller.

use super::geo::{wrap_180, wrap_360};

/// Exponential moving average filter for heading values.
///
/// Handles angle wrapping correctly (e.g., 350° → 10° transitions)
/// using shortest-path interpolation.
///
/// # Configuration
/// - `alpha = 1.0`: no filtering (pass-through)
/// - `alpha = 0.3`: moderate smoothing (default)
/// - `alpha = 0.0`: maximum smoothing (holds first heading indefinitely)
pub struct HeadingFilter {
    alpha: f32,
    prev_heading: Option<f32>,
}

impl HeadingFilter {
    /// Create a new HeadingFilter with the given smoothing factor.
    ///
    /// Alpha is clamped to [0.0, 1.0]. Lower alpha = more smoothing.
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            prev_heading: None,
        }
    }

    /// Apply the EMA filter to a raw heading value (degrees, 0-360).
    ///
    /// Returns the smoothed heading. The first call returns the raw heading unchanged.
    pub fn apply(&mut self, heading: f32) -> f32 {
        match self.prev_heading {
            None => {
                self.prev_heading = Some(heading);
                heading
            }
            Some(prev) => {
                // Angle-aware interpolation via shortest path
                let diff = wrap_180(heading - prev);
                let smoothed = wrap_360(prev + self.alpha * diff);
                self.prev_heading = Some(smoothed);
                smoothed
            }
        }
    }

    /// Reset the filter state, clearing the previous heading.
    pub fn reset(&mut self) {
        self.prev_heading = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_first_call_returns_raw_heading() {
        let mut filter = HeadingFilter::new(0.3);
        let result = filter.apply(45.0);
        assert!((result - 45.0).abs() < 0.001);
    }

    #[test]
    fn test_filter_smooths_noisy_heading() {
        let mut filter = HeadingFilter::new(0.3);

        // Establish baseline at 90°
        filter.apply(90.0);

        // Jump to 100° — should be smoothed
        let smoothed = filter.apply(100.0);
        // EMA: 90 + 0.3 * (100 - 90) = 93.0
        assert!(
            (smoothed - 93.0).abs() < 0.1,
            "Expected ~93.0, got {}",
            smoothed
        );

        // Another call at 100° — should move closer
        let smoothed2 = filter.apply(100.0);
        // EMA: 93 + 0.3 * (100 - 93) = 95.1
        assert!(
            (smoothed2 - 95.1).abs() < 0.1,
            "Expected ~95.1, got {}",
            smoothed2
        );
    }

    #[test]
    fn test_angle_wrapping_350_to_10() {
        let mut filter = HeadingFilter::new(0.3);

        // Start at 350°
        filter.apply(350.0);

        // Jump to 10° (shortest path is +20°, not -340°)
        let smoothed = filter.apply(10.0);
        // EMA: 350 + 0.3 * 20 = 356.0
        assert!(
            (smoothed - 356.0).abs() < 0.1,
            "Expected ~356.0 (wrapping correctly), got {}",
            smoothed
        );
    }

    #[test]
    fn test_alpha_one_passes_through() {
        let mut filter = HeadingFilter::new(1.0);

        filter.apply(90.0);
        let result = filter.apply(180.0);
        assert!(
            (result - 180.0).abs() < 0.001,
            "Alpha=1.0 should pass through, got {}",
            result
        );
    }

    #[test]
    fn test_alpha_zero_holds_first_heading() {
        let mut filter = HeadingFilter::new(0.0);

        filter.apply(90.0);
        let result = filter.apply(180.0);
        assert!(
            (result - 90.0).abs() < 0.001,
            "Alpha=0.0 should hold first heading, got {}",
            result
        );

        let result2 = filter.apply(270.0);
        assert!(
            (result2 - 90.0).abs() < 0.001,
            "Alpha=0.0 should still hold first heading, got {}",
            result2
        );
    }

    #[test]
    fn test_reset_clears_state() {
        let mut filter = HeadingFilter::new(0.3);

        filter.apply(90.0);
        filter.apply(100.0); // smoothed to ~93

        filter.reset();

        // After reset, next call should return raw heading
        let result = filter.apply(200.0);
        assert!(
            (result - 200.0).abs() < 0.001,
            "After reset, should return raw heading, got {}",
            result
        );
    }
}
