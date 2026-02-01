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

use core::cell::Cell;

use crate::devices::gps::GpsPosition;
use crate::subsystems::ahrs::SharedAhrsState;

// Re-export core heading types
pub use pico_trail_core::navigation::heading::{HeadingSource, HeadingSourceType};

/// Returns zero yaw offset (no compass calibration applied).
/// Used as default when no compass yaw offset provider is available.
fn zero_yaw_offset() -> f32 {
    0.0
}

/// Fused heading source combining AHRS yaw and GPS COG
///
/// This implementation selects the best heading source based on vehicle speed:
/// - Uses GPS COG when moving fast enough (speed >= threshold)
/// - Uses AHRS yaw when stationary or slow
/// - Falls back to GPS COG if AHRS is unhealthy
/// - Hysteresis prevents rapid switching near the speed threshold
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
///     || get_compass_yaw_offset(),  // yaw offset in radians
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
    /// Provider for compass yaw offset in radians (from Large Vehicle MagCal).
    /// Applied to AHRS yaw to correct heading for navigation.
    yaw_offset_provider: fn() -> f32,
    /// Hysteresis half-band for speed threshold (m/s)
    speed_hysteresis: f32,
    /// Track whether GPS COG is currently the active source
    using_gps_cog: Cell<bool>,
}

impl FusedHeadingSource {
    /// Default speed threshold for switching between AHRS and GPS COG
    pub const DEFAULT_SPEED_THRESHOLD: f32 = 1.0;

    /// Default hysteresis half-band (m/s). Set to 0.0 to disable hysteresis.
    pub const DEFAULT_SPEED_HYSTERESIS: f32 = 0.3;

    /// Create a new fused heading source
    ///
    /// # Arguments
    ///
    /// * `ahrs_state` - Reference to shared AHRS state
    /// * `gps_provider` - Function that returns current GPS position
    /// * `speed_threshold` - Speed (m/s) above which GPS COG is preferred
    /// * `yaw_offset_provider` - Function that returns compass yaw offset in radians
    pub const fn new(
        ahrs_state: &'static SharedAhrsState,
        gps_provider: fn() -> Option<GpsPosition>,
        speed_threshold: f32,
        yaw_offset_provider: fn() -> f32,
    ) -> Self {
        Self {
            ahrs_state,
            gps_provider,
            speed_threshold,
            yaw_offset_provider,
            speed_hysteresis: Self::DEFAULT_SPEED_HYSTERESIS,
            using_gps_cog: Cell::new(false),
        }
    }

    /// Create with default speed threshold (1.0 m/s) and no yaw offset
    pub const fn with_defaults(
        ahrs_state: &'static SharedAhrsState,
        gps_provider: fn() -> Option<GpsPosition>,
    ) -> Self {
        Self::new(
            ahrs_state,
            gps_provider,
            Self::DEFAULT_SPEED_THRESHOLD,
            zero_yaw_offset,
        )
    }

    /// Get AHRS yaw in degrees (0-360), corrected by compass yaw offset
    fn get_ahrs_heading(&self) -> Option<f32> {
        if !self.ahrs_state.is_healthy() {
            return None;
        }
        let yaw_rad = self.ahrs_state.get_yaw();
        let offset_rad = (self.yaw_offset_provider)();
        let corrected_yaw_rad = yaw_rad + offset_rad;
        let yaw_deg = corrected_yaw_rad.to_degrees();
        // Normalize to 0-360 using wrap_360 (no_std compatible)
        Some(super::wrap_360(yaw_deg))
    }

    /// Get GPS COG if available and speed is above effective threshold (with hysteresis)
    fn get_gps_heading(&self, gps: &GpsPosition) -> Option<f32> {
        let threshold = if self.using_gps_cog.get() {
            // Currently using GPS: switch away at lower threshold
            self.speed_threshold - self.speed_hysteresis
        } else {
            // Currently using AHRS: switch to GPS at higher threshold
            self.speed_threshold + self.speed_hysteresis
        };

        if gps.speed >= threshold {
            self.using_gps_cog.set(true);
            gps.course_over_ground
        } else {
            self.using_gps_cog.set(false);
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

        // Check GPS COG first (priority when moving, using hysteresis)
        if let Some(ref gps) = gps {
            let threshold = if self.using_gps_cog.get() {
                self.speed_threshold - self.speed_hysteresis
            } else {
                self.speed_threshold + self.speed_hysteresis
            };
            if gps.speed >= threshold && gps.course_over_ground.is_some() {
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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

        // Should use AHRS because GPS COG is None
        let heading = source.get_heading().unwrap();
        assert!(heading.abs() < 0.1, "Expected ~0°, got {}", heading);
        assert_eq!(source.source_type(), HeadingSourceType::Ahrs);
    }

    // ========== Edge Cases ==========

    #[test]
    fn test_speed_exactly_at_threshold() {
        // Setup: GPS speed exactly at base threshold (1.0 m/s)
        // With hysteresis 0.3, effective upper threshold = 1.3 when starting from AHRS
        set_test_gps(Some(create_gps(1.0, Some(120.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

        // Speed 1.0 is below the upper hysteresis threshold (1.3), so AHRS should be used
        let heading = source.get_heading().unwrap();
        assert!(
            (heading - 45.0).abs() < 0.1,
            "Expected ~45° from AHRS (speed within hysteresis band), got {}",
            heading
        );
        assert_eq!(source.source_type(), HeadingSourceType::Ahrs);
    }

    #[test]
    fn test_speed_just_below_threshold() {
        // Setup: GPS speed just below threshold (0.99 m/s)
        set_test_gps(Some(create_gps(0.99, Some(120.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

        // Should use GPS COG (0°) not AHRS (90°)
        assert_eq!(source.get_heading(), Some(0.0));
        assert_eq!(source.source_type(), HeadingSourceType::GpsCog);
    }

    #[test]
    fn test_gps_cog_360_degrees() {
        // Setup: GPS COG at 360° (same as 0° North)
        set_test_gps(Some(create_gps(5.0, Some(360.0))));
        TEST_AHRS_STATE.set_healthy(true);

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

        assert!(source.is_valid());
    }

    #[test]
    fn test_is_valid_gps_only() {
        // Setup: AHRS unhealthy, GPS available with COG
        set_test_gps(Some(create_gps(0.5, Some(45.0))));
        TEST_AHRS_STATE.set_healthy(false);

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

        assert!(source.is_valid());
    }

    #[test]
    fn test_is_valid_gps_without_cog() {
        // Setup: AHRS unhealthy, GPS available but no COG
        set_test_gps(Some(create_gps(2.0, None)));
        TEST_AHRS_STATE.set_healthy(false);

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

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
        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 2.0, zero_yaw_offset);

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

    // ========== Yaw Offset Tests ==========

    static mut TEST_YAW_OFFSET: f32 = 0.0;

    fn test_yaw_offset_provider() -> f32 {
        // Safety: Only used in single-threaded test context
        unsafe { TEST_YAW_OFFSET }
    }

    fn set_test_yaw_offset(offset: f32) {
        // Safety: Only used in single-threaded test context
        unsafe {
            TEST_YAW_OFFSET = offset;
        }
    }

    #[test]
    fn test_ahrs_heading_with_yaw_offset() {
        // Setup: AHRS yaw at 45°, compass offset +30° (0.5236 rad)
        set_test_gps(None);
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°
        set_test_yaw_offset(30.0_f32.to_radians());

        let source = FusedHeadingSource::new(
            &TEST_AHRS_STATE,
            test_gps_provider,
            1.0,
            test_yaw_offset_provider,
        );

        // Should return 45° + 30° = 75°
        let heading = source.get_heading().unwrap();
        assert!(
            (heading - 75.0).abs() < 0.5,
            "Expected ~75° (45° + 30° offset), got {}",
            heading
        );
    }

    #[test]
    fn test_ahrs_heading_with_negative_yaw_offset() {
        // Setup: AHRS yaw at 30°, compass offset -45° (-0.7854 rad)
        set_test_gps(None);
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, 30.0_f32.to_radians(), 0); // 30°
        set_test_yaw_offset(-45.0_f32.to_radians());

        let source = FusedHeadingSource::new(
            &TEST_AHRS_STATE,
            test_gps_provider,
            1.0,
            test_yaw_offset_provider,
        );

        // Should return 30° - 45° = -15° → wrapped to 345°
        let heading = source.get_heading().unwrap();
        assert!(
            (heading - 345.0).abs() < 0.5,
            "Expected ~345° (30° - 45° wrapped), got {}",
            heading
        );
    }

    #[test]
    fn test_gps_cog_ignores_yaw_offset() {
        // Setup: GPS moving, offset set. GPS COG should NOT be affected by offset.
        set_test_gps(Some(create_gps(2.0, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, 0.0, 0);
        set_test_yaw_offset(45.0_f32.to_radians());

        let source = FusedHeadingSource::new(
            &TEST_AHRS_STATE,
            test_gps_provider,
            1.0,
            test_yaw_offset_provider,
        );

        // GPS COG should be 90° (not affected by yaw offset)
        assert_eq!(source.get_heading(), Some(90.0));
    }

    // ========== Hysteresis Tests ==========

    #[test]
    fn test_hysteresis_gps_activates_at_upper_threshold() {
        // GPS COG should activate at threshold + hysteresis = 1.3 m/s
        set_test_gps(Some(create_gps(1.3, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

        // Speed 1.3 >= upper threshold (1.3) -> GPS COG
        assert_eq!(source.get_heading(), Some(90.0));
        assert_eq!(source.source_type(), HeadingSourceType::GpsCog);
    }

    #[test]
    fn test_hysteresis_gps_deactivates_at_lower_threshold() {
        // First, get into GPS COG state by providing high speed
        set_test_gps(Some(create_gps(2.0, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

        // Activate GPS COG
        assert_eq!(source.get_heading(), Some(90.0));

        // Now drop speed to 0.8 (above lower threshold 0.7) -> should still use GPS COG
        set_test_gps(Some(create_gps(0.8, Some(120.0))));
        assert_eq!(source.get_heading(), Some(120.0));

        // Drop speed to 0.6 (below lower threshold 0.7) -> should switch to AHRS
        set_test_gps(Some(create_gps(0.6, Some(120.0))));
        let heading = source.get_heading().unwrap();
        assert!(
            (heading - 45.0).abs() < 0.1,
            "Expected ~45° from AHRS after speed dropped below lower threshold, got {}",
            heading
        );
    }

    #[test]
    fn test_hysteresis_no_flip_flop_within_band() {
        // Speed oscillating between 0.8 and 1.2 m/s should NOT cause switching
        // when starting from AHRS (upper threshold = 1.3)
        set_test_gps(Some(create_gps(0.8, Some(90.0))));
        TEST_AHRS_STATE.set_healthy(true);
        TEST_AHRS_STATE.update_euler(0.0, 0.0, core::f32::consts::FRAC_PI_4, 0); // 45°

        let source =
            FusedHeadingSource::new(&TEST_AHRS_STATE, test_gps_provider, 1.0, zero_yaw_offset);

        // Initial: speed 0.8 < 1.3 (upper) -> AHRS
        let h1 = source.get_heading().unwrap();
        assert!((h1 - 45.0).abs() < 0.1, "Expected AHRS at 0.8 m/s");

        // Oscillate to 1.2 -> still < 1.3 -> AHRS
        set_test_gps(Some(create_gps(1.2, Some(90.0))));
        let h2 = source.get_heading().unwrap();
        assert!(
            (h2 - 45.0).abs() < 0.1,
            "Expected AHRS at 1.2 m/s (within band)"
        );

        // Back to 0.8 -> still AHRS
        set_test_gps(Some(create_gps(0.8, Some(90.0))));
        let h3 = source.get_heading().unwrap();
        assert!(
            (h3 - 45.0).abs() < 0.1,
            "Expected AHRS at 0.8 m/s (still within band)"
        );
    }

    #[test]
    fn test_hysteresis_default_value() {
        assert!(
            (FusedHeadingSource::DEFAULT_SPEED_HYSTERESIS - 0.3).abs() < 0.001,
            "Default speed hysteresis should be 0.3 m/s"
        );
    }
}
