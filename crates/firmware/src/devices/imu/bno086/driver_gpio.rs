//! BNO086 Driver with GPIO Support (INT/RST)
//!
//! This module extends the base BNO086 driver with:
//! - Interrupt-driven data acquisition (INT pin)
//! - Hardware reset recovery (RST pin)
//! - Timeout detection and automatic recovery
//!
//! # Requirements
//!
//! - T-x8mq2: BNO086 Driver Implementation (Phase 3)
//!
//! # Hardware Connections
//!
//! - INT: Active low, falling edge signals data ready
//! - RST: Active low, assert to reset sensor

use crate::communication::shtp::{ShtpChannel, ShtpError, ShtpPacket, ShtpTransport};
use crate::devices::traits::{QuaternionError, QuaternionReading, QuaternionSensor};
use nalgebra::Quaternion;

use super::gpio::{IntPin, RstPin};
use super::reports::{
    build_product_id_request, build_set_feature_command_for, ProductIdResponse, ReportId,
    RotationVectorReport,
};

/// Maximum consecutive resets before giving up
const MAX_CONSECUTIVE_RESETS: u32 = 3;

/// Maximum consecutive errors before triggering reset
const MAX_ERRORS_BEFORE_RESET: u32 = 3;

/// INT timeout in milliseconds
const INT_TIMEOUT_MS: u64 = 500;

/// Reset pulse duration in milliseconds
/// BNO086 requires minimum 10ms reset pulse, using 100ms for reliability
const RESET_PULSE_MS: u64 = 100;

/// Post-reset boot wait in milliseconds
/// BNO086 boot time is ~650ms per datasheet, using 1000ms for safety
const POST_RESET_BOOT_MS: u64 = 1000;

/// Maximum packets to read during initialization
const MAX_INIT_PACKETS: usize = 10;

/// Default report interval for 100Hz (microseconds)
const DEFAULT_REPORT_INTERVAL_US: u32 = 10_000;

// =============================================================================
// Time Abstraction Helpers
// =============================================================================

async fn delay_ms(ms: u64) {
    embassy_time::Timer::after_millis(ms).await;
}

fn timestamp_us() -> u64 {
    embassy_time::Instant::now().as_micros()
}

/// Async timeout wrapper using select
async fn with_timeout<F, T>(timeout_ms: u64, future: F) -> Result<T, ()>
where
    F: core::future::Future<Output = T>,
{
    use embassy_futures::select::{select, Either};
    use embassy_time::Timer;

    match select(future, Timer::after_millis(timeout_ms)).await {
        Either::First(result) => Ok(result),
        Either::Second(_) => Err(()),
    }
}

/// BNO086 Driver Configuration
#[derive(Debug, Clone, Copy)]
pub struct Bno086GpioConfig {
    /// Report interval in microseconds (default: 10000 = 100Hz)
    pub report_interval_us: u32,

    /// INT timeout in milliseconds (default: 500ms)
    pub int_timeout_ms: u64,

    /// Enable hardware reset on consecutive errors (default: true)
    pub enable_auto_reset: bool,
}

impl Default for Bno086GpioConfig {
    fn default() -> Self {
        Self {
            report_interval_us: DEFAULT_REPORT_INTERVAL_US,
            int_timeout_ms: INT_TIMEOUT_MS,
            enable_auto_reset: true,
        }
    }
}

/// Driver state for error recovery tracking
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DriverState {
    /// Not initialized
    Uninitialized,
    /// Normal operation
    Running,
    /// Currently resetting sensor
    Resetting,
    /// Too many consecutive resets, sensor unhealthy
    Failed,
}

/// BNO086 Driver with GPIO Support
///
/// Provides interrupt-driven quaternion reading and hardware reset recovery.
///
/// # Type Parameters
///
/// * `T` - SHTP transport (e.g., `ShtpI2c`)
/// * `INT` - Interrupt pin implementing `IntPin`
/// * `RST` - Reset pin implementing `RstPin`
///
/// # Example
///
/// ```ignore
/// use pico_trail::devices::imu::bno086::{
///     Bno086DriverWithGpio, Bno086GpioConfig, EmbassyIntPin, EmbassyRstPin
/// };
/// use pico_trail::communication::shtp::ShtpI2c;
///
/// let transport = ShtpI2c::new(i2c, 0x4A);
/// let int_pin = EmbassyIntPin::new(gpio_input);
/// let rst_pin = EmbassyRstPin::new(gpio_output);
///
/// let mut driver = Bno086DriverWithGpio::new(
///     transport,
///     int_pin,
///     rst_pin,
///     Bno086GpioConfig::default()
/// );
/// driver.init().await?;
///
/// loop {
///     let reading = driver.read_quaternion().await?;
///     // Use reading.quaternion...
/// }
/// ```
pub struct Bno086DriverWithGpio<T, INT, RST>
where
    T: ShtpTransport,
    INT: IntPin,
    RST: RstPin,
{
    /// SHTP transport layer
    transport: T,

    /// Interrupt pin (data ready signal)
    int_pin: INT,

    /// Reset pin (hardware reset)
    rst_pin: RST,

    /// Configuration
    config: Bno086GpioConfig,

    /// Current driver state
    state: DriverState,

    /// Last quaternion reading
    last_quaternion: Quaternion<f32>,

    /// Last accuracy estimate (radians)
    last_accuracy: f32,

    /// Last update timestamp (microseconds)
    last_update_us: u64,

    /// Consecutive error count (resets to 0 on successful read)
    error_count: u32,

    /// Consecutive reset count (resets to 0 on successful init)
    reset_count: u32,

    /// Product ID information (populated after init)
    product_id: Option<ProductIdResponse>,
}

impl<T, INT, RST> Bno086DriverWithGpio<T, INT, RST>
where
    T: ShtpTransport,
    INT: IntPin,
    RST: RstPin,
{
    /// Create a new BNO086 driver with GPIO support
    ///
    /// # Arguments
    ///
    /// * `transport` - SHTP transport (e.g., ShtpI2c)
    /// * `int_pin` - Interrupt pin for data ready detection
    /// * `rst_pin` - Reset pin for hardware reset
    /// * `config` - Driver configuration
    pub fn new(transport: T, int_pin: INT, rst_pin: RST, config: Bno086GpioConfig) -> Self {
        Self {
            transport,
            int_pin,
            rst_pin,
            config,
            state: DriverState::Uninitialized,
            last_quaternion: Quaternion::identity(),
            last_accuracy: core::f32::consts::PI,
            last_update_us: 0,
            error_count: 0,
            reset_count: 0,
            product_id: None,
        }
    }

    /// Initialize the BNO086 sensor
    ///
    /// Performs hardware reset, waits for boot, and configures Rotation Vector output.
    /// Retries initialization up to MAX_CONSECUTIVE_RESETS times on failure.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Initialization successful
    /// * `Err(QuaternionError)` - Initialization failed after all retries
    pub async fn init(&mut self) -> Result<(), QuaternionError> {
        for attempt in 0..MAX_CONSECUTIVE_RESETS {
            if attempt > 0 {
                crate::log_info!(
                    "BNO086: Init retry attempt {}/{}",
                    attempt + 1,
                    MAX_CONSECUTIVE_RESETS
                );
            }

            match self.init_internal().await {
                Ok(()) => {
                    // Initialization successful
                    self.state = DriverState::Running;
                    self.error_count = 0;
                    self.reset_count = 0;
                    crate::log_info!("BNO086: Initialized successfully");
                    return Ok(());
                }
                Err(_e) => {
                    crate::log_warn!("BNO086: Init attempt {} failed: {:?}", attempt + 1, _e);
                    // Wait before retry
                    delay_ms(500).await;
                }
            }
        }

        crate::log_error!("BNO086: All init attempts failed");
        self.state = DriverState::Failed;
        Err(QuaternionError::NotInitialized)
    }

    /// Internal initialization logic (called by init with retry wrapper)
    async fn init_internal(&mut self) -> Result<(), QuaternionError> {
        // Perform hardware reset
        self.hardware_reset().await;

        // Wait for INT signal (boot complete)
        // Use wait_for_int() which checks is_low() first - handles case where
        // INT is already LOW after MCU reset (BNO086 was not reset)
        if self.wait_for_int().await.is_err() {
            crate::log_warn!("BNO086: No INT after reset, continuing anyway");
        }

        // Reset transport state
        self.transport.reset();

        // Wait additional boot time
        delay_ms(POST_RESET_BOOT_MS).await;

        // Clear buffer with retry
        self.clear_buffer_with_retry().await?;

        // Additional delay after clearing buffer for I2C stability
        delay_ms(50).await;

        // Request and read Product ID
        self.request_product_id().await?;
        delay_ms(50).await;
        self.read_product_id().await?;

        // Configure Rotation Vector report
        self.configure_rotation_vector().await?;
        delay_ms(50).await;

        Ok(())
    }

    /// Perform hardware reset using RST pin
    ///
    /// Asserts RST low for 100ms, then releases and waits for boot.
    pub async fn hardware_reset(&mut self) {
        crate::log_debug!("BNO086: Hardware reset");

        self.state = DriverState::Resetting;

        // Assert reset (active low)
        self.rst_pin.set_low();
        delay_ms(RESET_PULSE_MS).await;

        // Release reset
        self.rst_pin.set_high();
        delay_ms(RESET_PULSE_MS).await;
    }

    /// Wait for INT signal with timeout
    ///
    /// First checks if INT is already LOW (data ready). If not, waits for
    /// falling edge with timeout.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - INT signal received (data ready)
    /// * `Err(QuaternionError::Timeout)` - Timeout waiting for INT
    async fn wait_for_int(&mut self) -> Result<(), QuaternionError> {
        // First check if INT is already LOW (data is already ready)
        if self.int_pin.is_low() {
            return Ok(());
        }

        // Wait for falling edge with timeout
        match with_timeout(
            self.config.int_timeout_ms,
            self.int_pin.wait_for_falling_edge(),
        )
        .await
        {
            Ok(_) => Ok(()),
            Err(_) => {
                crate::log_warn!("BNO086: INT timeout ({}ms)", self.config.int_timeout_ms);
                Err(QuaternionError::Timeout)
            }
        }
    }

    /// Read quaternion with interrupt-driven acquisition
    ///
    /// This method:
    /// 1. Waits for INT falling edge (data ready)
    /// 2. Reads and parses SHTP packet
    /// 3. Extracts Rotation Vector report
    /// 4. Returns quaternion reading
    ///
    /// On errors, triggers automatic recovery if enabled.
    async fn read_quaternion_internal(&mut self) -> Result<QuaternionReading, QuaternionError> {
        // Check state
        match self.state {
            DriverState::Uninitialized => return Err(QuaternionError::NotInitialized),
            DriverState::Resetting => return Err(QuaternionError::Resetting),
            DriverState::Failed => return Err(QuaternionError::NotInitialized),
            DriverState::Running => {}
        }

        // Wait for INT with timeout
        if let Err(e) = self.wait_for_int().await {
            self.handle_error().await;
            return Err(e);
        }

        // Read packets until we get a Rotation Vector report
        for _ in 0..5 {
            match self.read_and_process().await {
                Ok(true) => {
                    // Successfully read Rotation Vector
                    self.error_count = 0;
                    return Ok(QuaternionReading::new(
                        self.last_quaternion,
                        self.last_accuracy,
                        self.last_update_us,
                    ));
                }
                Ok(false) => {
                    // Got different packet type, continue
                }
                Err(e) => {
                    self.handle_error().await;
                    return Err(e);
                }
            }
        }

        // No Rotation Vector in recent packets
        self.handle_error().await;
        Err(QuaternionError::Timeout)
    }

    /// Handle error and potentially trigger recovery
    async fn handle_error(&mut self) {
        self.error_count = self.error_count.saturating_add(1);

        crate::log_warn!(
            "BNO086: Error count {} (max before reset: {})",
            self.error_count,
            MAX_ERRORS_BEFORE_RESET
        );

        // Check if we should trigger auto-reset
        if self.config.enable_auto_reset && self.error_count >= MAX_ERRORS_BEFORE_RESET {
            self.trigger_recovery().await;
        }
    }

    /// Trigger recovery sequence
    ///
    /// Performs hardware reset and re-initialization.
    /// Protected against reset loops.
    async fn trigger_recovery(&mut self) {
        self.reset_count = self.reset_count.saturating_add(1);

        crate::log_warn!(
            "BNO086: Triggering recovery (reset count: {}/{})",
            self.reset_count,
            MAX_CONSECUTIVE_RESETS
        );

        // Check for reset loop
        if self.reset_count >= MAX_CONSECUTIVE_RESETS {
            crate::log_error!("BNO086: Too many consecutive resets, marking as failed");
            self.state = DriverState::Failed;
            return;
        }

        // Perform hardware reset
        self.hardware_reset().await;

        // Wait for boot
        delay_ms(POST_RESET_BOOT_MS).await;

        // Try to reinitialize
        match self.reinitialize().await {
            Ok(()) => {
                crate::log_info!("BNO086: Recovery successful");
                self.error_count = 0;
                self.state = DriverState::Running;
            }
            Err(_e) => {
                crate::log_error!("BNO086: Recovery failed: {:?}", _e);
                // Will retry on next read
            }
        }
    }

    /// Re-initialize sensor after reset (lightweight init)
    async fn reinitialize(&mut self) -> Result<(), QuaternionError> {
        // Reset transport
        self.transport.reset();

        // Wait for INT (boot complete)
        // Check is_low() first to handle case where INT is already asserted
        if !self.int_pin.is_low() {
            let _ = with_timeout(
                self.config.int_timeout_ms,
                self.int_pin.wait_for_falling_edge(),
            )
            .await;
        }

        // Additional delay for I2C bus stabilization after boot
        delay_ms(100).await;

        // Clear buffer
        self.clear_buffer().await;

        // Configure Rotation Vector report
        self.configure_rotation_vector().await?;
        delay_ms(100).await;

        Ok(())
    }

    /// Clear sensor's output buffer with retry on I2C errors
    ///
    /// Wait for INT before each read to ensure sensor is ready.
    /// Retries on I2C errors instead of immediately failing.
    async fn clear_buffer_with_retry(&mut self) -> Result<(), QuaternionError> {
        let mut packet = ShtpPacket::<280>::new();
        let mut consecutive_errors = 0;
        const MAX_CONSECUTIVE_ERRORS: u32 = 5;

        for _i in 0..MAX_INIT_PACKETS {
            // Wait for INT or timeout before reading
            // Check is_low() first to handle case where INT is already asserted
            if !self.int_pin.is_low() {
                let _ = with_timeout(200, self.int_pin.wait_for_falling_edge()).await;
            }

            match self.transport.read_packet(&mut packet).await {
                Ok(()) => {
                    consecutive_errors = 0;
                    crate::log_trace!("BNO086: clear_buffer read OK");
                }
                Err(ShtpError::NoData) => {
                    // No more data to read, success
                    crate::log_debug!("BNO086: clear_buffer complete");
                    return Ok(());
                }
                Err(_e) => {
                    consecutive_errors += 1;
                    crate::log_debug!(
                        "BNO086: clear_buffer error {:?} ({}/{})",
                        _e,
                        consecutive_errors,
                        MAX_CONSECUTIVE_ERRORS
                    );
                    if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                        crate::log_warn!("BNO086: clear_buffer too many errors");
                        return Err(QuaternionError::I2cError);
                    }
                    // Wait longer on error before retry
                    delay_ms(50).await;
                    continue;
                }
            }
            // Delay between successful reads
            delay_ms(20).await;
        }

        Ok(())
    }

    /// Clear sensor's output buffer (simple version for recovery)
    ///
    /// Used during recovery where we don't want to fail on errors.
    async fn clear_buffer(&mut self) {
        let mut packet = ShtpPacket::<280>::new();

        for _ in 0..MAX_INIT_PACKETS {
            // Wait for INT or timeout before reading
            // Check is_low() first to handle case where INT is already asserted
            if !self.int_pin.is_low() {
                let _ = with_timeout(200, self.int_pin.wait_for_falling_edge()).await;
            }

            match self.transport.read_packet(&mut packet).await {
                Ok(()) => {}
                Err(ShtpError::NoData) => break,
                Err(_) => {
                    delay_ms(50).await;
                    continue;
                }
            }
            delay_ms(20).await;
        }
    }

    /// Send Product ID request
    async fn request_product_id(&mut self) -> Result<(), QuaternionError> {
        let payload = build_product_id_request();
        let mut packet = ShtpPacket::<280>::with_header(ShtpChannel::Control, 0);
        packet
            .set_payload(&payload)
            .map_err(|_| QuaternionError::ProtocolError)?;

        self.transport
            .write_packet(&packet)
            .await
            .map_err(|_| QuaternionError::I2cError)?;

        Ok(())
    }

    /// Read Product ID response
    ///
    /// Wait for INT before each read and use longer delays for I2C stability.
    async fn read_product_id(&mut self) -> Result<(), QuaternionError> {
        let mut packet = ShtpPacket::<280>::new();

        for _ in 0..MAX_INIT_PACKETS {
            // Wait for INT or timeout before reading
            // Check is_low() first to handle case where INT is already asserted
            if !self.int_pin.is_low() {
                let _ = with_timeout(100, self.int_pin.wait_for_falling_edge()).await;
            }

            match self.transport.read_packet(&mut packet).await {
                Ok(()) => {
                    if let Some(response) = ProductIdResponse::parse(packet.payload()) {
                        crate::log_info!(
                            "BNO086: Product ID - SW {}.{}.{}",
                            response.sw_version_major,
                            response.sw_version_minor,
                            response.sw_version_patch
                        );
                        self.product_id = Some(response);
                        return Ok(());
                    }
                }
                Err(ShtpError::TransportError) => {
                    // Log and retry instead of immediate failure
                    crate::log_debug!("BNO086: Product ID read transport error, retrying...");
                }
                Err(_) => {}
            }
            // Increased delay for I2C stability (was 5ms)
            delay_ms(20).await;
        }

        // Product ID not critical, continue anyway
        Ok(())
    }

    /// Configure Game Rotation Vector report at specified rate
    ///
    /// Uses Game Rotation Vector (0x08) which is more commonly available
    /// on BNO086 without magnetometer calibration.
    async fn configure_rotation_vector(&mut self) -> Result<(), QuaternionError> {
        // Use Game Rotation Vector (0x08) - more reliable than Rotation Vector (0x05)
        // because it doesn't require magnetometer calibration
        let payload = build_set_feature_command_for(
            ReportId::GameRotationVector as u8,
            self.config.report_interval_us,
        );
        let mut packet = ShtpPacket::<280>::with_header(ShtpChannel::Control, 0);
        packet
            .set_payload(&payload)
            .map_err(|_| QuaternionError::ProtocolError)?;

        self.transport
            .write_packet(&packet)
            .await
            .map_err(|_| QuaternionError::I2cError)?;

        crate::log_debug!(
            "BNO086: Configured Game Rotation Vector at {}us interval",
            self.config.report_interval_us
        );

        Ok(())
    }

    /// Read and process a single packet
    ///
    /// Accepts both direct reports (0x05, 0x08) and batched reports (0xFB)
    async fn read_and_process(&mut self) -> Result<bool, QuaternionError> {
        let mut packet = ShtpPacket::<280>::new();

        match self.transport.read_packet(&mut packet).await {
            Ok(()) => {
                if packet.channel == ShtpChannel::InputReport as u8 {
                    let payload = packet.payload();
                    if payload.is_empty() {
                        return Ok(false);
                    }

                    let report_id = payload[0];

                    // Direct reports: 0x05 (Rotation Vector) or 0x08 (Game Rotation Vector)
                    if report_id == 0x05 || report_id == 0x08 {
                        if let Some(report) = RotationVectorReport::parse_any(payload) {
                            self.last_quaternion = report.to_quaternion();
                            self.last_accuracy = report.accuracy_radians();
                            self.last_update_us = timestamp_us();
                            return Ok(true);
                        }
                    }

                    // 0xFB = Base Timestamp Reference - contains batched reports
                    // Format: [0xFB][4-byte timestamp][report][2-byte delta][report]...
                    if report_id == 0xFB && payload.len() > 7 {
                        let mut offset = 5; // Skip 0xFB + 4-byte timestamp
                        let mut first_report = true;
                        let mut got_quaternion = false;

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
                                0x01 => 10, // Accelerometer
                                0x02 => 10, // Gyroscope Calibrated
                                0x03 => 10, // Magnetometer
                                0x04 => 10, // Linear Acceleration
                                0x05 => 14, // Rotation Vector (with accuracy)
                                0x08 => 12, // Game Rotation Vector (no accuracy)
                                _ => {
                                    offset += 1;
                                    continue;
                                }
                            };

                            if offset + report_size > payload.len() {
                                break;
                            }

                            // Parse quaternion reports (0x05 or 0x08)
                            if embedded_id == 0x05 || embedded_id == 0x08 {
                                if let Some(report) =
                                    RotationVectorReport::parse_any(&payload[offset..])
                                {
                                    self.last_quaternion = report.to_quaternion();
                                    self.last_accuracy = report.accuracy_radians();
                                    self.last_update_us = timestamp_us();
                                    got_quaternion = true;
                                }
                            }

                            offset += report_size;
                        }

                        if got_quaternion {
                            return Ok(true);
                        }
                    }
                }
                Ok(false)
            }
            Err(ShtpError::TransportError) => Err(QuaternionError::I2cError),
            Err(_) => Err(QuaternionError::ProtocolError),
        }
    }

    /// Get Product ID (if available)
    pub fn product_id(&self) -> Option<&ProductIdResponse> {
        self.product_id.as_ref()
    }

    /// Get current error count
    pub fn error_count(&self) -> u32 {
        self.error_count
    }

    /// Get current reset count
    pub fn reset_count(&self) -> u32 {
        self.reset_count
    }

    /// Check if driver is in failed state
    pub fn is_failed(&self) -> bool {
        self.state == DriverState::Failed
    }

    /// Get driver configuration
    pub fn config(&self) -> &Bno086GpioConfig {
        &self.config
    }

    /// Release transport and GPIO pins
    pub fn release(self) -> (T, INT, RST) {
        (self.transport, self.int_pin, self.rst_pin)
    }
}

impl<T, INT, RST> QuaternionSensor for Bno086DriverWithGpio<T, INT, RST>
where
    T: ShtpTransport,
    INT: IntPin,
    RST: RstPin,
{
    async fn read_quaternion(&mut self) -> Result<QuaternionReading, QuaternionError> {
        self.read_quaternion_internal().await
    }

    fn is_healthy(&self) -> bool {
        self.state == DriverState::Running && self.error_count < MAX_ERRORS_BEFORE_RESET
    }

    fn last_update_us(&self) -> u64 {
        self.last_update_us
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::shtp::{ShtpError, ShtpPacket, ShtpTransport};

    /// Mock INT pin for testing
    struct MockIntPin {
        is_low: bool,
    }

    impl MockIntPin {
        fn new(is_low: bool) -> Self {
            Self { is_low }
        }
    }

    impl IntPin for MockIntPin {
        async fn wait_for_falling_edge(&mut self) {
            // Simulate immediate return
        }

        fn is_low(&self) -> bool {
            self.is_low
        }
    }

    /// Mock RST pin for testing
    struct MockRstPin {
        is_low: bool,
    }

    impl MockRstPin {
        fn new() -> Self {
            Self { is_low: false }
        }
    }

    impl RstPin for MockRstPin {
        fn set_low(&mut self) {
            self.is_low = true;
        }

        fn set_high(&mut self) {
            self.is_low = false;
        }
    }

    /// Mock transport for testing
    struct MockTransport {
        read_count: usize,
        packets: Vec<Vec<u8>>,
    }

    impl MockTransport {
        fn new() -> Self {
            Self {
                read_count: 0,
                packets: Vec::new(),
            }
        }

        fn _add_rotation_vector_packet(&mut self) {
            // Identity quaternion
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
    }

    impl ShtpTransport for MockTransport {
        async fn read_packet<const N: usize>(
            &mut self,
            packet: &mut ShtpPacket<N>,
        ) -> Result<(), ShtpError> {
            if self.read_count < self.packets.len() {
                let data = &self.packets[self.read_count];
                self.read_count += 1;

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
            _packet: &ShtpPacket<N>,
        ) -> Result<(), ShtpError> {
            Ok(())
        }

        fn reset(&mut self) {
            self.read_count = 0;
        }
    }

    #[test]
    fn test_driver_gpio_new() {
        let transport = MockTransport::new();
        let int_pin = MockIntPin::new(true);
        let rst_pin = MockRstPin::new();

        let driver =
            Bno086DriverWithGpio::new(transport, int_pin, rst_pin, Bno086GpioConfig::default());

        assert!(!driver.is_healthy());
        assert_eq!(driver.error_count(), 0);
        assert_eq!(driver.reset_count(), 0);
        assert!(!driver.is_failed());
    }

    #[test]
    fn test_gpio_config_default() {
        let config = Bno086GpioConfig::default();
        assert_eq!(config.report_interval_us, 10_000);
        assert_eq!(config.int_timeout_ms, 500);
        assert!(config.enable_auto_reset);
    }

    #[tokio::test]
    async fn test_read_quaternion_not_initialized() {
        let transport = MockTransport::new();
        let int_pin = MockIntPin::new(true);
        let rst_pin = MockRstPin::new();

        let mut driver =
            Bno086DriverWithGpio::new(transport, int_pin, rst_pin, Bno086GpioConfig::default());

        let result = driver.read_quaternion().await;
        assert!(matches!(result, Err(QuaternionError::NotInitialized)));
    }

    #[test]
    fn test_mock_rst_pin() {
        let mut pin = MockRstPin::new();
        assert!(!pin.is_low);

        pin.set_low();
        assert!(pin.is_low);

        pin.set_high();
        assert!(!pin.is_low);
    }
}
