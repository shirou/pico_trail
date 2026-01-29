//! Heading source abstraction for navigation
//!
//! This module provides a unified interface for obtaining heading information
//! from multiple sources (AHRS yaw, GPS Course Over Ground).
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────┐     ┌─────────────────┐
//! │ SharedAhrsState │     │  GPS Provider   │
//! │ (yaw, healthy)  │     │ (position, COG) │
//! └────────┬────────┘     └────────┬────────┘
//!          │                       │
//!          └───────────┬───────────┘
//!                      ▼
//!          ┌─────────────────────┐
//!          │  FusedHeadingSource │
//!          │  (HeadingSource)    │
//!          └──────────┬──────────┘
//!                     │
//!                     ▼ get_heading() -> Option<f32>
//! ```
//!
//! # Heading Source Selection
//!
//! `FusedHeadingSource` selects the heading source based on vehicle speed:
//! - **GPS COG** when speed >= threshold and COG is available (moving)
//! - **AHRS yaw** when stationary or GPS COG unavailable
//! - Falls back to GPS COG if AHRS is unhealthy

use crate::devices::gps::GpsPosition;
use crate::subsystems::ahrs::SharedAhrsState;

// Re-export core heading types
pub use pico_trail_core::navigation::heading::{HeadingSource, HeadingSourceType};

/// Fused heading source combining AHRS yaw and GPS COG
///
/// This implementation selects the best heading source based on vehicle speed:
/// - Uses GPS COG when moving fast enough (speed >= threshold)
/// - Uses AHRS yaw when stationary or slow
/// - Falls back to GPS COG if AHRS is unhealthy
///
/// # Example
///
/// ```ignore
/// use crate::subsystems::navigation::heading::FusedHeadingSource;
/// use crate::subsystems::ahrs::AHRS_STATE;
///
/// let heading_source = FusedHeadingSource::new(
///     &AHRS_STATE,
///     || get_gps_position(),
///     1.0,  // speed threshold m/s
/// );
///
/// if let Some(heading) = heading_source.get_heading() {
///     // Use heading for navigation
/// }
/// ```
pub struct FusedHeadingSource {
    ahrs_state: &'static SharedAhrsState,
    gps_provider: fn() -> Option<GpsPosition>,
    speed_threshold: f32,
}

impl FusedHeadingSource {
    /// Default speed threshold for switching between AHRS and GPS COG
    pub const DEFAULT_SPEED_THRESHOLD: f32 = 1.0;

    /// Create a new fused heading source
    ///
    /// # Arguments
    ///
    /// * `ahrs_state` - Reference to shared AHRS state
    /// * `gps_provider` - Function that returns current GPS position
    /// * `speed_threshold` - Speed (m/s) above which GPS COG is preferred
    pub const fn new(
        ahrs_state: &'static SharedAhrsState,
        gps_provider: fn() -> Option<GpsPosition>,
        speed_threshold: f32,
    ) -> Self {
        Self {
            ahrs_state,
            gps_provider,
            speed_threshold,
        }
    }

    /// Create with default speed threshold (1.0 m/s)
    pub const fn with_defaults(
        ahrs_state: &'static SharedAhrsState,
        gps_provider: fn() -> Option<GpsPosition>,
    ) -> Self {
        Self::new(ahrs_state, gps_provider, Self::DEFAULT_SPEED_THRESHOLD)
    }

    /// Get AHRS yaw in degrees (0-360)
    fn get_ahrs_heading(&self) -> Option<f32> {
        if !self.ahrs_state.is_healthy() {
            return None;
        }
        let yaw_rad = self.ahrs_state.get_yaw();
        let yaw_deg = yaw_rad.to_degrees();
        // Normalize to 0-360 using wrap_360 (no_std compatible)
        Some(super::wrap_360(yaw_deg))
    }

    /// Get GPS COG if available and speed is above threshold
    fn get_gps_heading(&self, gps: &GpsPosition) -> Option<f32> {
        if gps.speed >= self.speed_threshold {
            gps.course_over_ground
        } else {
            None
        }
    }
}

impl HeadingSource for FusedHeadingSource {
    fn get_heading(&self) -> Option<f32> {
        let gps = (self.gps_provider)();

        // Priority 1: GPS COG when moving fast enough
        if let Some(ref gps) = gps {
            if let Some(cog) = self.get_gps_heading(gps) {
                return Some(cog);
            }
        }

        // Priority 2: AHRS yaw when stationary or GPS COG unavailable
        if let Some(ahrs_heading) = self.get_ahrs_heading() {
            return Some(ahrs_heading);
        }

        // Priority 3: GPS COG even if below speed threshold (last resort)
        if let Some(ref gps) = gps {
            if let Some(cog) = gps.course_over_ground {
                return Some(cog);
            }
        }

        // No valid heading available
        None
    }

    fn is_valid(&self) -> bool {
        // Valid if AHRS is healthy OR GPS is available
        if self.ahrs_state.is_healthy() {
            return true;
        }
        if let Some(gps) = (self.gps_provider)() {
            // GPS COG is typically only valid when moving
            return gps.course_over_ground.is_some();
        }
        false
    }

    fn source_type(&self) -> HeadingSourceType {
        let gps = (self.gps_provider)();

        // Check GPS COG first (priority when moving)
        if let Some(ref gps) = gps {
            if gps.speed >= self.speed_threshold && gps.course_over_ground.is_some() {
                return HeadingSourceType::GpsCog;
            }
        }

        // Check AHRS
        if self.ahrs_state.is_healthy() {
            return HeadingSourceType::Ahrs;
        }

        // Check GPS COG below threshold (fallback)
        if let Some(ref gps) = gps {
            if gps.course_over_ground.is_some() {
                return HeadingSourceType::GpsCog;
            }
        }

        HeadingSourceType::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::gps::GpsFixType;

    // Test SharedAhrsState for unit tests
    static TEST_AHRS_STATE: SharedAhrsState = SharedAhrsState::new();

    // Test GPS position storage
    static mut TEST_GPS: Option<GpsPosition> = None;

    fn test_gps_provider() -> Option<GpsPosition> {
        // Safety: Only used in single-threaded test context
        unsafe { TEST_GPS }
    }

    fn set_test_gps(gps: Option<GpsPosition>) {
        // Safety: Only used in single-threaded test context
        unsafe {
            TEST_GPS = gps;
        }
    }

    fn create_gps(speed: f32, cog: Option<f32>) -> GpsPosition {
        GpsPosition {
            latitude: 35.0,
            longitude: 139.0,
            altitude: 10.0,
            speed,
            course_over_ground: cog,
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        }
    }

    #[test]
    fn test_gps_cog_when_moving() {
        // Setup: GPS moving at 2.0 m/s with COG 90°, AHRS healthy with yaw 45°
        set_test_gps(Some(create_gps(2.0, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Should use GPS COG because speed > threshold
        assert_eq!(source.get_heading(), Some(90.0));
        assert_eq!(source.source_type(), HeadingSourceType::GpsCog);
        assert!(source.is_valid());
    }

    #[test]
    fn test_ahrs_when_stationary() {
        // Setup: GPS stationary (speed 0.5 m/s), AHRS healthy with yaw 45°
        set_test_gps(Some(create_gps(0.5, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Should use AHRS because speed < threshold
        let heading = source.get_heading().unwrap();
        assert!(
            (heading - 45.0).abs() < 0.1,
            "Expected ~45°, got {}",
            heading
        );
        assert_eq!(source.source_type(), HeadingSourceType::Ahrs);
        assert!(source.is_valid());
    }

    #[test]
    fn test_gps_fallback_when_ahrs_unhealthy() {
        // Setup: GPS stationary with COG, AHRS unhealthy
        set_test_gps(Some(create_gps(0.5, Some(180.0))));
        TEST_AHRS_STATE.set_healthy(false);

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Should fall back to GPS COG even below threshold
        assert_eq!(source.get_heading(), Some(180.0));
        assert_eq!(source.source_type(), HeadingSourceType::GpsCog);
        assert!(source.is_valid());
    }

    #[test]
    fn test_no_heading_when_both_unavailable() {
        // Setup: No GPS, AHRS unhealthy
        set_test_gps(None);
        TEST_AHRS_STATE.set_healthy(false);

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        assert_eq!(source.get_heading(), None);
        assert_eq!(source.source_type(), HeadingSourceType::None);
        assert!(!source.is_valid());
    }

    #[test]
    fn test_ahrs_only_no_gps() {
        // Setup: No GPS, AHRS healthy with yaw 270°
        set_test_gps(None);
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, -core::f32::consts::FRAC_PI_2, 0); // -90° = 270°

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        let heading = source.get_heading().unwrap();
        assert!(
            (heading - 270.0).abs() < 0.1,
            "Expected ~270°, got {}",
            heading
        );
        assert_eq!(source.source_type(), HeadingSourceType::Ahrs);
        assert!(source.is_valid());
    }

    #[test]
    fn test_gps_no_cog_uses_ahrs() {
        // Setup: GPS moving but no COG (can happen during signal issues), AHRS healthy
        set_test_gps(Some(create_gps(2.0, None)));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, 0.0, 0); // 0° (North)

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Should use AHRS because GPS COG is None
        let heading = source.get_heading().unwrap();
        assert!(heading.abs() < 0.1, "Expected ~0°, got {}", heading);
        assert_eq!(source.source_type(), HeadingSourceType::Ahrs);
    }

    // ========== Edge Cases ==========

    #[test]
    fn test_speed_exactly_at_threshold() {
        // Setup: GPS speed exactly at threshold (1.0 m/s)
        set_test_gps(Some(create_gps(1.0, Some(120.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // At exactly threshold, GPS COG should be used (>= comparison)
        assert_eq!(source.get_heading(), Some(120.0));
        assert_eq!(source.source_type(), HeadingSourceType::GpsCog);
    }

    #[test]
    fn test_speed_just_below_threshold() {
        // Setup: GPS speed just below threshold (0.99 m/s)
        set_test_gps(Some(create_gps(0.99, Some(120.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Below threshold, AHRS should be used
        let heading = source.get_heading().unwrap();
        assert!(
            (heading - 45.0).abs() < 0.1,
            "Expected ~45° from AHRS, got {}",
            heading
        );
        assert_eq!(source.source_type(), HeadingSourceType::Ahrs);
    }

    #[test]
    fn test_ahrs_yaw_wrap_around_360() {
        // Setup: AHRS yaw near 360° (should wrap properly)
        set_test_gps(None);
        TEST_AHRS_STATE.set_healthy(true);
        // Set yaw to -5° (355° when normalized)
        TEST_AHRS_STATE.update_euler(0.0, 0.0, -5.0_f32.to_radians(), 0);

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        let heading = source.get_heading().unwrap();
        // Should be normalized to 355°
        assert!(
            (heading - 355.0).abs() < 0.5,
            "Expected ~355°, got {}",
            heading
        );
    }

    #[test]
    fn test_ahrs_yaw_negative_180() {
        // Setup: AHRS yaw at -180° (should become 180°)
        set_test_gps(None);
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, -core::f32::consts::PI, 0); // -180°

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        let heading = source.get_heading().unwrap();
        // Should be normalized to 180°
        assert!(
            (heading - 180.0).abs() < 0.5,
            "Expected ~180°, got {}",
            heading
        );
    }

    #[test]
    fn test_gps_cog_zero_degrees() {
        // Setup: GPS COG at exactly 0° (North)
        set_test_gps(Some(create_gps(5.0, Some(0.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_2, 0); // 90°

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Should use GPS COG (0°) not AHRS (90°)
        assert_eq!(source.get_heading(), Some(0.0));
        assert_eq!(source.source_type(), HeadingSourceType::GpsCog);
    }

    #[test]
    fn test_gps_cog_360_degrees() {
        // Setup: GPS COG at 360° (same as 0° North)
        set_test_gps(Some(create_gps(5.0, Some(360.0))));
        TEST_AHRS_STATE.set_healthy(true);

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Should use GPS COG
        assert_eq!(source.get_heading(), Some(360.0));
        assert_eq!(source.source_type(), HeadingSourceType::GpsCog);
    }

    #[test]
    fn test_zero_speed_with_cog() {
        // Setup: GPS speed = 0, but COG is still reported (some GPS receivers do this)
        set_test_gps(Some(create_gps(0.0, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, 0.0, 0); // 0° (North)

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Should use AHRS (speed below threshold) not GPS COG
        let heading = source.get_heading().unwrap();
        assert!(
            heading.abs() < 0.1,
            "Expected ~0° from AHRS, got {}",
            heading
        );
        assert_eq!(source.source_type(), HeadingSourceType::Ahrs);
    }

    #[test]
    fn test_is_valid_ahrs_only() {
        // Setup: AHRS healthy, no GPS
        set_test_gps(None);
        TEST_AHRS_STATE.set_healthy(true);

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        assert!(source.is_valid());
    }

    #[test]
    fn test_is_valid_gps_only() {
        // Setup: AHRS unhealthy, GPS available with COG
        set_test_gps(Some(create_gps(0.5, Some(45.0))));
        TEST_AHRS_STATE.set_healthy(false);

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        assert!(source.is_valid());
    }

    #[test]
    fn test_is_valid_gps_without_cog() {
        // Setup: AHRS unhealthy, GPS available but no COG
        set_test_gps(Some(create_gps(2.0, None)));
        TEST_AHRS_STATE.set_healthy(false);

        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0);

        // Without AHRS or COG, should not be valid
        assert!(!source.is_valid());
    }

    #[test]
    fn test_default_speed_threshold() {
        assert!(
            (FusedHeadingSource::DEFAULT_SPEED_THRESHOLD - 1.0).abs() < 0.001,
            "Default speed threshold should be 1.0 m/s"
        );
    }

    #[test]
    fn test_with_defaults_constructor() {
        set_test_gps(Some(create_gps(2.0, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);

        let source = FusedHeadingSource::with_defaults(&TEST_AHRS_STATE, test_gps_provider);

        // Should work the same as new() with 1.0 threshold
        assert_eq!(source.get_heading(), Some(90.0));
    }

    #[test]
    fn test_custom_speed_threshold() {
        // Setup: GPS speed 1.5 m/s, custom threshold 2.0 m/s
        set_test_gps(Some(create_gps(1.5, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        // With threshold 2.0, 1.5 m/s is below threshold -> use AHRS
        let source = FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 2.0);

        let heading = source.get_heading().unwrap();
        assert!(
            (heading - 45.0).abs() < 0.1,
            "Expected ~45° from AHRS with high threshold, got {}",
            heading
        );
        assert_eq!(source.source_type(), HeadingSourceType::Ahrs);
    }

    #[test]
    fn test_heading_source_type_debug() {
        // Test that HeadingSourceType implements Debug properly
        let ahrs = HeadingSourceType::Ahrs;
        let gps = HeadingSourceType::GpsCog;
        let none = HeadingSourceType::None;

        assert_eq!(format!("{:?}", ahrs), "Ahrs");
        assert_eq!(format!("{:?}", gps), "GpsCog");
        assert_eq!(format!("{:?}", none), "None");
    }

    #[test]
    fn test_heading_source_type_clone() {
        let source_type = HeadingSourceType::Ahrs;
        let cloned = source_type;
        assert_eq!(source_type, cloned);
    }
}
