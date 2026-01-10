//! BNO086 Driver Implementation
//!
//! Core driver implementation for reading quaternion data via SHTP protocol.
//!
//! This driver is transport-agnostic and works with any `ShtpTransport` implementation.
//! GPIO operations for INT/RST pins are handled in Phase 3.
//!
//! # Features
//!
//! - Quaternion output via Rotation Vector report (100Hz)
//! - On-chip sensor fusion (ARM Cortex-M0+)
//! - No external EKF required
//!
//! # References
//!
//! - AN-srhcj: BNO086 IMU Integration Analysis
//! - T-x8mq2: BNO086 Driver Implementation Task

use crate::communication::shtp::{ShtpChannel, ShtpError, ShtpPacket, ShtpTransport};
use crate::devices::traits::{QuaternionError, QuaternionReading, QuaternionSensor};
use nalgebra::Quaternion;

use super::reports::{build_product_id_request, ProductIdResponse, RotationVectorReport};

/// Maximum consecutive errors before marking sensor unhealthy
const MAX_CONSECUTIVE_ERRORS: u32 = 3;

/// Default report interval for 100Hz (microseconds)
const DEFAULT_REPORT_INTERVAL_US: u32 = 10_000;

/// Maximum packets to read during initialization (to clear buffer)
const MAX_INIT_PACKETS: usize = 10;

// =============================================================================
// Time Abstraction Helpers
// =============================================================================

/// Async delay in milliseconds
#[cfg(feature = "embassy")]
async fn delay_ms(ms: u64) {
    embassy_time::Timer::after_millis(ms).await;
}

#[cfg(not(feature = "embassy"))]
async fn delay_ms(_ms: u64) {
    // No-op for host tests
}

/// Get current timestamp in microseconds
#[cfg(feature = "embassy")]
fn timestamp_us() -> u64 {
    embassy_time::Instant::now().as_micros()
}

#[cfg(not(feature = "embassy"))]
fn timestamp_us() -> u64 {
    0 // Host test stub
}

/// BNO086 Driver Configuration
#[derive(Debug, Clone, Copy)]
pub struct Bno086Config {
    /// Report interval in microseconds (default: 10000 = 100Hz)
    pub report_interval_us: u32,
}

impl Default for Bno086Config {
    fn default() -> Self {
        Self {
            report_interval_us: DEFAULT_REPORT_INTERVAL_US,
        }
    }
}

/// BNO086 9-Axis IMU Driver with On-Chip Sensor Fusion
///
/// Implements `QuaternionSensor` trait for direct quaternion output from the BNO086.
/// Uses SHTP protocol for communication.
///
/// # Type Parameters
///
/// * `T` - Any type implementing `ShtpTransport` (e.g., `ShtpI2c`)
///
/// # Example
///
/// ```ignore
/// use pico_trail::devices::imu::bno086::Bno086Driver;
/// use pico_trail::communication::shtp::ShtpI2c;
///
/// let transport = ShtpI2c::new(i2c, 0x4A);
/// let mut driver = Bno086Driver::new(transport, Bno086Config::default());
/// driver.init().await?;
///
/// let reading = driver.read_quaternion().await?;
/// ```
pub struct Bno086Driver<T>
where
    T: ShtpTransport,
{
    /// SHTP transport layer
    transport: T,

    /// Driver configuration
    config: Bno086Config,

    /// Last quaternion reading
    last_quaternion: Quaternion<f32>,

    /// Last accuracy estimate (radians)
    last_accuracy: f32,

    /// Last update timestamp (microseconds)
    last_update_us: u64,

    /// Health status
    healthy: bool,

    /// Consecutive error count
    error_count: u32,

    /// Initialization complete flag
    initialized: bool,

    /// Product ID information (populated after init)
    product_id: Option<ProductIdResponse>,
}

impl<T> Bno086Driver<T>
where
    T: ShtpTransport,
{
    /// Create a new BNO086 driver
    ///
    /// # Arguments
    ///
    /// * `transport` - SHTP transport implementation
    /// * `config` - Driver configuration
    ///
    /// # Note
    ///
    /// The driver is not initialized after construction. Call `init()` before
    /// reading quaternion data.
    pub fn new(transport: T, config: Bno086Config) -> Self {
        Self {
            transport,
            config,
            last_quaternion: Quaternion::identity(),
            last_accuracy: core::f32::consts::PI,
            last_update_us: 0,
            healthy: false,
            error_count: 0,
            initialized: false,
            product_id: None,
        }
    }

    /// Initialize the BNO086 sensor
    ///
    /// This method:
    /// 1. Waits for sensor boot (100ms)
    /// 2. Reads and discards initial packets
    /// 3. Requests Product ID to verify communication
    /// 4. Configures Rotation Vector report at specified rate
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Initialization successful
    /// * `Err(QuaternionError)` - Initialization failed
    pub async fn init(&mut self) -> Result<(), QuaternionError> {
        // Reset transport state
        self.transport.reset();

        // Wait for sensor boot
        delay_ms(100).await;

        // Read and discard initial packets (clear buffer)
        self.clear_buffer().await;

        // Request Product ID
        self.request_product_id().await?;

        // Wait for response
        delay_ms(10).await;

        // Read Product ID response
        self.read_product_id().await?;

        // Configure Rotation Vector report
        self.configure_rotation_vector().await?;

        // Wait for sensor to start generating reports
        // BNO086 needs time to stabilize after configuration
        delay_ms(200).await;

        // Try to get first valid reading to confirm sensor is working
        #[cfg(feature = "pico2_w")]
        crate::log_info!("Waiting for first quaternion data...");

        let mut got_data = false;
        #[allow(unused_variables)]
        for attempt in 0..30 {
            let mut packet = ShtpPacket::<128>::new();
            if self.transport.read_packet(&mut packet).await.is_ok()
                && packet.channel == ShtpChannel::InputReport as u8
            {
                let payload = packet.payload();
                if !payload.is_empty() {
                    let report_id = payload[0];
                    // Check for quaternion data (direct or in batched report)
                    if report_id == 0x05 || report_id == 0x08 || report_id == 0xFB {
                        #[cfg(feature = "pico2_w")]
                        crate::log_info!(
                            "Got first data (report 0x{:02X}) after {} attempts",
                            report_id,
                            attempt + 1
                        );
                        got_data = true;
                        break;
                    }
                }
            }
            delay_ms(20).await;
        }

        if !got_data {
            #[cfg(feature = "pico2_w")]
            crate::log_warn!("No quaternion data received during init, sensor may need more time");
        }

        self.initialized = true;
        self.healthy = true;
        self.error_count = 0;

        Ok(())
    }

    /// Clear the sensor's output buffer by reading packets
    async fn clear_buffer(&mut self) {
        let mut packet = ShtpPacket::<128>::new();

        for _ in 0..MAX_INIT_PACKETS {
            if self.transport.read_packet(&mut packet).await.is_err() {
                break;
            }
            // Small delay between reads
            delay_ms(1).await;
        }
    }

    /// Send Product ID request
    async fn request_product_id(&mut self) -> Result<(), QuaternionError> {
        let payload = build_product_id_request();
        let mut packet = ShtpPacket::<128>::with_header(ShtpChannel::Control, 0);
        packet
            .set_payload(&payload)
            .map_err(|_| QuaternionError::ProtocolError)?;

        self.transport
            .write_packet(&packet)
            .await
            .map_err(|_| QuaternionError::I2cError)?;

        Ok(())
    }

    /// Read and parse Product ID response
    async fn read_product_id(&mut self) -> Result<(), QuaternionError> {
        let mut packet = ShtpPacket::<128>::new();

        // Try to read product ID response (may need multiple reads)
        for _ in 0..MAX_INIT_PACKETS {
            match self.transport.read_packet(&mut packet).await {
                Ok(()) => {
                    if let Some(response) = ProductIdResponse::parse(packet.payload()) {
                        self.product_id = Some(response);
                        return Ok(());
                    }
                }
                Err(ShtpError::TransportError) => {
                    return Err(QuaternionError::I2cError);
                }
                Err(_) => {
                    // Continue trying
                }
            }
            delay_ms(5).await;
        }

        // Failed to get Product ID - sensor may still work
        // Log warning but don't fail initialization
        Ok(())
    }

    /// Configure Rotation Vector report
    async fn configure_rotation_vector(&mut self) -> Result<(), QuaternionError> {
        use super::reports::{build_set_feature_command_for, ReportId};

        // Try enabling Game Rotation Vector (0x08) - doesn't need magnetometer
        #[cfg(feature = "pico2_w")]
        crate::log_info!(
            "Enabling Game Rotation Vector (0x08) at {}us interval",
            self.config.report_interval_us
        );

        let payload = build_set_feature_command_for(
            ReportId::GameRotationVector as u8,
            self.config.report_interval_us,
        );

        let mut packet = ShtpPacket::<128>::with_header(ShtpChannel::Control, 0);
        packet
            .set_payload(&payload)
            .map_err(|_| QuaternionError::ProtocolError)?;

        self.transport
            .write_packet(&packet)
            .await
            .map_err(|_| QuaternionError::I2cError)?;

        #[cfg(feature = "pico2_w")]
        crate::log_info!("Feature command sent, waiting for response...");

        // Wait for command to be processed
        delay_ms(100).await;

        // Read responses to see if we get a Feature Response (0xFC)
        let mut packet = ShtpPacket::<128>::new();
        for _ in 0..10 {
            if self.transport.read_packet(&mut packet).await.is_ok() {
                let payload_data = packet.payload();
                let report_id = if !payload_data.is_empty() {
                    payload_data[0]
                } else {
                    0
                };

                #[cfg(feature = "pico2_w")]
                crate::log_debug!(
                    "Response: ch={} len={} report_id=0x{:02X}",
                    packet.channel,
                    payload_data.len(),
                    report_id
                );

                // 0xFC is Get Feature Response - confirms feature is enabled
                if report_id == ReportId::GetFeatureResponse as u8 {
                    #[cfg(feature = "pico2_w")]
                    crate::log_info!("Feature enabled successfully!");
                    return Ok(());
                }
            }
            delay_ms(20).await;
        }

        #[cfg(feature = "pico2_w")]
        crate::log_warn!("No feature response received, sensor may not be configured");

        Ok(())
    }

    /// Read a single packet and process if it's a Rotation Vector report
    ///
    /// This is the polling-mode read function. In Phase 3, this will be
    /// replaced with interrupt-driven reading.
    async fn read_and_process(&mut self) -> Result<bool, QuaternionError> {
        let mut packet = ShtpPacket::<128>::new();

        match self.transport.read_packet(&mut packet).await {
            Ok(()) => {
                // Check if this is a Rotation Vector or Game Rotation Vector report
                if packet.channel == ShtpChannel::InputReport as u8 {
                    let payload = packet.payload();
                    if !payload.is_empty() {
                        let report_id = payload[0];

                        // 0x05 = Rotation Vector, 0x08 = Game Rotation Vector (direct)
                        if report_id == 0x05 || report_id == 0x08 {
                            if let Some(report) = RotationVectorReport::parse_any(payload) {
                                self.last_quaternion = report.to_quaternion();
                                self.last_accuracy = report.accuracy_radians();
                                self.last_update_us = timestamp_us();
                                self.error_count = 0;
                                self.healthy = true;
                                return Ok(true);
                            }
                        }

                        // 0xFB = Base Timestamp Reference - contains batched reports
                        // Format: [0xFB][4-byte timestamp][report][2-byte delta][report]...
                        if report_id == 0xFB && payload.len() > 7 {
                            // Parse embedded reports by structure
                            // offset 0: 0xFB
                            // offset 1-4: base timestamp (4 bytes)
                            // offset 5+: first embedded report (NO time delta before first)
                            // After each report: [2-byte delta][next report]...
                            let mut offset = 5;
                            let mut first_report = true;

                            while offset < payload.len() {
                                // Skip 2-byte time delta (except for first report)
                                if !first_report {
                                    offset += 2;
                                    if offset >= payload.len() {
                                        break;
                                    }
                                }
                                first_report = false;

                                let embedded_id = payload[offset];
                                let report_size = match embedded_id {
                                    0x01..=0x04 => 10, // Accel/Gyro/Mag/LinAccel
                                    0x05 => 14,        // Rotation Vector (with accuracy)
                                    0x08 => 12,        // Game Rotation Vector (no accuracy)
                                    _ => {
                                        // Unknown report - skip 1 byte and try again
                                        offset += 1;
                                        continue;
                                    }
                                };

                                // Check if we have enough data for this report
                                if offset + report_size > payload.len() {
                                    break;
                                }

                                // Parse quaternion reports
                                if embedded_id == 0x05 || embedded_id == 0x08 {
                                    if let Some(report) =
                                        RotationVectorReport::parse_any(&payload[offset..])
                                    {
                                        self.last_quaternion = report.to_quaternion();
                                        self.last_accuracy = report.accuracy_radians();
                                        self.last_update_us = timestamp_us();
                                        self.error_count = 0;
                                        self.healthy = true;
                                        return Ok(true);
                                    }
                                }

                                // Move to next report
                                offset += report_size;
                            }
                        }
                    }
                }
                // Packet received but not a Rotation Vector report
                Ok(false)
            }
            Err(ShtpError::NoData) => {
                // No data available - this is normal in polling mode without INT
                // Don't increment error count, just return false to try again
                Ok(false)
            }
            Err(ShtpError::TransportError) => {
                self.increment_error();
                Err(QuaternionError::I2cError)
            }
            Err(ShtpError::InvalidHeader) => {
                self.increment_error();
                Err(QuaternionError::ProtocolError)
            }
            Err(ShtpError::PayloadTooLarge) => {
                self.increment_error();
                Err(QuaternionError::ProtocolError)
            }
            Err(_) => {
                self.increment_error();
                Err(QuaternionError::ProtocolError)
            }
        }
    }

    /// Increment error count and update health status
    fn increment_error(&mut self) {
        self.error_count = self.error_count.saturating_add(1);
        if self.error_count >= MAX_CONSECUTIVE_ERRORS {
            self.healthy = false;
        }
    }

    /// Get the Product ID response (if available)
    pub fn product_id(&self) -> Option<&ProductIdResponse> {
        self.product_id.as_ref()
    }

    /// Check if the driver has been initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get the current configuration
    pub fn config(&self) -> &Bno086Config {
        &self.config
    }

    /// Get the current error count
    pub fn error_count(&self) -> u32 {
        self.error_count
    }

    /// Release the transport layer
    pub fn release(self) -> T {
        self.transport
    }
}

impl<T> QuaternionSensor for Bno086Driver<T>
where
    T: ShtpTransport,
{
    async fn read_quaternion(&mut self) -> Result<QuaternionReading, QuaternionError> {
        if !self.initialized {
            return Err(QuaternionError::NotInitialized);
        }

        // Try to read a rotation vector report
        // In polling mode without INT, we may need to poll multiple times
        // At 100Hz, data arrives every 10ms, so poll for up to 50ms with delays
        for _ in 0..10 {
            match self.read_and_process().await {
                Ok(true) => {
                    return Ok(QuaternionReading::new(
                        self.last_quaternion,
                        self.last_accuracy,
                        self.last_update_us,
                    ));
                }
                Ok(false) => {
                    // Continue reading - got a different packet type or no data yet
                    delay_ms(5).await;
                }
                Err(_) => {
                    // Transient errors - wait longer and retry instead of failing
                    delay_ms(10).await;
                }
            }
        }

        // No rotation vector report found in recent packets
        // Return last known value if we have one
        if self.last_update_us > 0 {
            Ok(QuaternionReading::new(
                self.last_quaternion,
                self.last_accuracy,
                self.last_update_us,
            ))
        } else {
            Err(QuaternionError::Timeout)
        }
    }

    fn is_healthy(&self) -> bool {
        self.healthy && self.initialized
    }

    fn last_update_us(&self) -> u64 {
        self.last_update_us
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::shtp::{ShtpError, ShtpPacket, ShtpTransport};

    /// Mock transport for testing
    struct MockTransport {
        read_count: usize,
        packets: Vec<Vec<u8>>,
        write_log: Vec<Vec<u8>>,
    }

    impl MockTransport {
        fn new() -> Self {
            Self {
                read_count: 0,
                packets: Vec::new(),
                write_log: Vec::new(),
            }
        }

        #[allow(dead_code)]
        fn add_rotation_vector_packet(&mut self) {
            // Simulated Rotation Vector report
            // Identity quaternion: w=1, x=y=z=0
            let payload: Vec<u8> = vec![
                0x05, // Report ID
                0x01, // Sequence
                0x03, // Status
                0x00, 0x00, // Delay
                0x00, 0x00, // Q_i = 0
                0x00, 0x00, // Q_j = 0
                0x00, 0x00, // Q_k = 0
                0x00, 0x40, // Q_real = 16384
                0x00, 0x00, // Accuracy
            ];
            self.packets.push(payload);
        }

        #[allow(dead_code)]
        fn add_product_id_packet(&mut self) {
            let payload: Vec<u8> = vec![
                0xF8, // Report ID
                0x01, // Reset cause
                0x03, // SW version major
                0x02, // SW version minor
                0x01, 0x02, 0x03, 0x04, // SW part number
                0x05, 0x06, 0x07, 0x08, // SW build number
                0x09, 0x0A, // SW version patch
                0x00, 0x00, // Padding
            ];
            self.packets.push(payload);
        }
    }

    impl ShtpTransport for MockTransport {
        async fn read_packet<const N: usize>(
            &mut self,
            packet: &mut ShtpPacket<N>,
        ) -> Result<(), ShtpError> {
            if self.read_count < self.packets.len() {
                let data = &self.packets[self.read_count];
                self.read_count += 1;

                // Set up packet with InputReport channel
                packet.channel = ShtpChannel::InputReport as u8;
                packet.sequence = self.read_count as u8;
                packet.length = (4 + data.len()) as u16;
                packet.payload_len = data.len().min(N);
                packet.payload[..packet.payload_len].copy_from_slice(&data[..packet.payload_len]);

                Ok(())
            } else {
                Err(ShtpError::Timeout)
            }
        }

        async fn write_packet<const N: usize>(
            &mut self,
            packet: &ShtpPacket<N>,
        ) -> Result<(), ShtpError> {
            self.write_log.push(packet.payload().to_vec());
            Ok(())
        }

        fn reset(&mut self) {
            self.read_count = 0;
        }
    }

    #[test]
    fn test_bno086_driver_new() {
        let transport = MockTransport::new();
        let driver = Bno086Driver::new(transport, Bno086Config::default());

        assert!(!driver.is_initialized());
        assert!(!driver.is_healthy());
        assert_eq!(driver.error_count(), 0);
    }

    #[test]
    fn test_bno086_config_default() {
        let config = Bno086Config::default();
        assert_eq!(config.report_interval_us, 10_000); // 100Hz
    }

    #[test]
    fn test_bno086_driver_not_initialized() {
        let transport = MockTransport::new();
        let driver = Bno086Driver::new(transport, Bno086Config::default());

        // Should not be able to get healthy status without initialization
        assert!(!driver.is_healthy());
    }

    #[tokio::test]
    async fn test_bno086_read_quaternion_not_initialized() {
        let transport = MockTransport::new();
        let mut driver = Bno086Driver::new(transport, Bno086Config::default());

        let result = driver.read_quaternion().await;
        assert!(matches!(result, Err(QuaternionError::NotInitialized)));
    }
}
