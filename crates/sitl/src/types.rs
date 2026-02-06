use core::fmt;

/// Vehicle identifier (matches MAVLink system ID range).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VehicleId(pub u8);

impl fmt::Display for VehicleId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vehicle({})", self.0)
    }
}

/// GPS fix type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpsFixType {
    NoFix,
    Fix2D,
    Fix3D,
    DGps,
    RtkFloat,
    RtkFixed,
}

/// IMU sensor data.
#[derive(Debug, Clone)]
pub struct ImuData {
    /// Accelerometer reading in m/s² (body frame, [x, y, z]).
    pub accel_mss: [f32; 3],
    /// Gyroscope reading in rad/s (body frame, [x, y, z]).
    pub gyro_rads: [f32; 3],
    /// Temperature in degrees Celsius.
    pub temperature_c: f32,
}

/// GPS sensor data.
#[derive(Debug, Clone)]
pub struct GpsData {
    /// Latitude in degrees.
    pub lat_deg: f64,
    /// Longitude in degrees.
    pub lon_deg: f64,
    /// Altitude in meters above sea level.
    pub alt_m: f32,
    /// Ground speed in m/s.
    pub speed_ms: f32,
    /// Course over ground in degrees.
    pub course_deg: f32,
    /// Fix type.
    pub fix_type: GpsFixType,
    /// Number of visible satellites.
    pub satellites: u8,
    /// Horizontal dilution of precision.
    pub hdop: f32,
}

/// Compass (magnetometer) sensor data.
#[derive(Debug, Clone)]
pub struct CompassData {
    /// Magnetic field in Gauss (body frame, [x, y, z]).
    pub mag_gauss: [f32; 3],
}

/// Barometer sensor data.
#[derive(Debug, Clone)]
pub struct BarometerData {
    /// Atmospheric pressure in hPa.
    pub pressure_hpa: f32,
    /// Temperature in degrees Celsius.
    pub temperature_c: f32,
}

/// Aggregated sensor data from a simulator.
#[derive(Debug, Clone)]
pub struct SensorData {
    /// Timestamp in microseconds (simulation time).
    pub timestamp_us: u64,
    /// Vehicle that produced this data.
    pub vehicle_id: VehicleId,
    /// IMU data (accelerometer + gyroscope).
    pub imu: Option<ImuData>,
    /// GPS data.
    pub gps: Option<GpsData>,
    /// Compass (magnetometer) data.
    pub compass: Option<CompassData>,
    /// Barometer data.
    pub barometer: Option<BarometerData>,
}

/// Actuator commands sent to a simulator.
#[derive(Debug, Clone)]
pub struct ActuatorCommands {
    /// Timestamp in microseconds (simulation time).
    pub timestamp_us: u64,
    /// Target vehicle.
    pub vehicle_id: VehicleId,
    /// Motor outputs, normalized -1.0 to 1.0.
    pub motors: Vec<f32>,
    /// Servo outputs, normalized -1.0 to 1.0.
    pub servos: Vec<f32>,
}
