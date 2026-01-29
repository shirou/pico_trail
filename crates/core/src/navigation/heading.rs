//! Heading source abstraction for navigation
//!
//! Provides a unified interface for obtaining heading information
//! from multiple sources (AHRS yaw, GPS Course Over Ground).
//!
//! Platform-specific implementations (FusedHeadingSource) are in the firmware crate.

/// Type of heading source currently being used
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeadingSourceType {
    /// Heading from AHRS yaw (stationary or low speed)
    Ahrs,
    /// Heading from GPS Course Over Ground (moving)
    GpsCog,
    /// No valid heading source available
    None,
}

/// Provides heading information for navigation
///
/// This trait abstracts the source of heading data, allowing different
/// implementations for various sensor configurations.
pub trait HeadingSource {
    /// Returns current heading in degrees (0-360, 0 = North)
    ///
    /// Returns `None` if no valid heading is available.
    fn get_heading(&self) -> Option<f32>;

    /// Returns true if heading source is healthy and providing valid data
    fn is_valid(&self) -> bool;

    /// Returns the current heading source type for telemetry/debugging
    fn source_type(&self) -> HeadingSourceType;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heading_source_type_equality() {
        assert_eq!(HeadingSourceType::Ahrs, HeadingSourceType::Ahrs);
        assert_eq!(HeadingSourceType::GpsCog, HeadingSourceType::GpsCog);
        assert_eq!(HeadingSourceType::None, HeadingSourceType::None);
        assert_ne!(HeadingSourceType::Ahrs, HeadingSourceType::GpsCog);
        assert_ne!(HeadingSourceType::Ahrs, HeadingSourceType::None);
    }

    #[test]
    fn test_heading_source_type_debug() {
        extern crate std;
        use std::format;
        assert_eq!(format!("{:?}", HeadingSourceType::Ahrs), "Ahrs");
        assert_eq!(format!("{:?}", HeadingSourceType::GpsCog), "GpsCog");
        assert_eq!(format!("{:?}", HeadingSourceType::None), "None");
    }

    #[test]
    fn test_heading_source_type_clone() {
        let source_type = HeadingSourceType::Ahrs;
        let cloned = source_type;
        assert_eq!(source_type, cloned);
    }
}
