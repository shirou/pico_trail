use std::collections::{HashMap, HashSet};
use std::env;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

/// Pin type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum PinType {
    Input, // Digital input
    #[default]
    Output, // Digital output (default for actuators)
    Adc,   // Analog input
    I2c,   // I2C peripheral pin (SDA/SCL)
}

/// Pull mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum PullMode {
    #[default]
    None,
    PullUp,
    PullDown,
}

/// Output mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum OutputMode {
    #[default]
    PushPull,
    OpenDrain,
}

/// Pin speed configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Speed {
    Low,
    #[default]
    Medium,
    High,
    VeryHigh,
}

/// Pin configuration with modifiers
#[derive(Debug, Clone, Copy)]
struct PinConfig {
    gpio: u8,
    pin_type: PinType,
    pull: PullMode,
    output_mode: OutputMode,
    speed: Speed,
}

impl PinConfig {
    fn new(gpio: u8) -> Self {
        Self {
            gpio,
            pin_type: PinType::default(),
            pull: PullMode::default(),
            output_mode: OutputMode::default(),
            speed: Speed::default(),
        }
    }

    /// Parse modifiers from hwdef line parts
    fn with_modifiers(
        mut self,
        modifiers: &[&str],
        path: &Path,
        line_num: usize,
    ) -> Result<Self, String> {
        for modifier in modifiers {
            match *modifier {
                // Pin type modifiers
                "INPUT" => self.pin_type = PinType::Input,
                "OUTPUT" => self.pin_type = PinType::Output,
                "ADC" => self.pin_type = PinType::Adc,
                "I2C" => self.pin_type = PinType::I2c,

                // Pull mode modifiers
                "PULLUP" => self.pull = PullMode::PullUp,
                "PULLDOWN" => self.pull = PullMode::PullDown,

                // Output mode modifiers (unsupported on RP2350, but parsed)
                "OPENDRAIN" => {
                    println!("cargo:warning={}:{}: OPENDRAIN modifier is not supported on RP2350 (ignored)",
                             path.display(), line_num);
                    self.output_mode = OutputMode::OpenDrain;
                }

                // Speed modifiers (unsupported on RP2350, but parsed)
                "SPEED_LOW" => {
                    println!("cargo:warning={}:{}: SPEED_LOW modifier is not supported on RP2350 (ignored)",
                             path.display(), line_num);
                    self.speed = Speed::Low;
                }
                "SPEED_MEDIUM" => {
                    println!("cargo:warning={}:{}: SPEED_MEDIUM modifier is not supported on RP2350 (ignored)",
                             path.display(), line_num);
                    self.speed = Speed::Medium;
                }
                "SPEED_HIGH" => {
                    println!("cargo:warning={}:{}: SPEED_HIGH modifier is not supported on RP2350 (ignored)",
                             path.display(), line_num);
                    self.speed = Speed::High;
                }
                "SPEED_VERY_HIGH" => {
                    println!("cargo:warning={}:{}: SPEED_VERY_HIGH modifier is not supported on RP2350 (ignored)",
                             path.display(), line_num);
                    self.speed = Speed::VeryHigh;
                }

                _ => {
                    return Err(format!(
                        "{}:{}: Unknown pin modifier '{}'",
                        path.display(),
                        line_num,
                        modifier
                    ));
                }
            }
        }

        // Validate modifier combinations
        if self.pull == PullMode::PullUp && matches!(self.pull, PullMode::PullDown) {
            return Err(format!(
                "{}:{}: Cannot specify both PULLUP and PULLDOWN",
                path.display(),
                line_num
            ));
        }

        Ok(self)
    }
}

/// Parsed hwdef.dat configuration
#[derive(Debug)]
struct HwDefConfig {
    platform: String,
    motor_count: usize,
    motor_pins: Vec<MotorPinPair>,
    servo_count: usize,
    servo_pins: Vec<ServoPins>,
    esc_count: usize,
    esc_pins: Vec<EscPins>,
    stepper_count: usize,
    stepper_pins: Vec<StepperPins>,
    buzzer: Option<PinConfig>,
    led: Option<PinConfig>,
    battery_adc: Option<PinConfig>,
    all_pins: HashMap<String, PinConfig>,
}

/// Motor pin pair (IN1, IN2) for H-bridge control
#[derive(Debug, Clone, Copy)]
struct MotorPinPair {
    in1: PinConfig,
    in2: PinConfig,
}

/// Servo pins (single PWM pin)
#[derive(Debug, Clone, Copy)]
struct ServoPins {
    pwm: PinConfig,
}

/// ESC pins (single PWM pin for brushless motor control)
#[derive(Debug, Clone, Copy)]
struct EscPins {
    pwm: PinConfig,
}

/// Stepper motor pins (STEP, DIR, ENABLE, optional microstepping)
#[derive(Debug, Clone, Copy)]
struct StepperPins {
    step: PinConfig,
    dir: PinConfig,
    enable: PinConfig,
    ms1: Option<PinConfig>,
}

/// Helper to parse GPIO number and modifiers
fn parse_pin_config(
    path: &Path,
    line_num: usize,
    key: &str,
    parts: &[&str],
) -> Result<PinConfig, String> {
    if parts.is_empty() {
        return Err(format!(
            "{}:{}: Missing GPIO number for {}",
            path.display(),
            line_num,
            key
        ));
    }

    let gpio = parts[0].parse::<u8>().map_err(|_| {
        format!(
            "{}:{}: Invalid GPIO number '{}' for {}",
            path.display(),
            line_num,
            parts[0],
            key
        )
    })?;

    let modifiers = &parts[1..];
    PinConfig::new(gpio).with_modifiers(modifiers, path, line_num)
}

/// Parse hwdef.dat file with include support
fn parse_hwdef(path: &Path) -> Result<HwDefConfig, String> {
    let mut include_chain = HashSet::new();
    parse_hwdef_with_includes(path, &mut include_chain)
}

/// Internal parser with include recursion tracking
fn parse_hwdef_with_includes(
    path: &Path,
    include_chain: &mut HashSet<PathBuf>,
) -> Result<HwDefConfig, String> {
    // Check for circular includes
    let canonical_path = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());

    if include_chain.contains(&canonical_path) {
        return Err(format!(
            "Circular include detected: {} -> {}",
            include_chain
                .iter()
                .map(|p| p.display().to_string())
                .collect::<Vec<_>>()
                .join(" -> "),
            canonical_path.display()
        ));
    }

    include_chain.insert(canonical_path.clone());

    // Ensure rebuild when this file changes (important for included files)
    println!("cargo:rerun-if-changed={}", path.display());

    let content = fs::read_to_string(path)
        .map_err(|e| format!("Failed to read hwdef file {}: {}", path.display(), e))?;

    let mut platform = None;
    let mut motor_count = None;
    let mut servo_count = None;
    let mut esc_count = None;
    let mut stepper_count = None;
    let mut pins: HashMap<String, PinConfig> = HashMap::new();

    for (line_num, line) in content.lines().enumerate() {
        let line_num = line_num + 1; // 1-indexed for user-facing errors

        // Remove inline comments (everything after '#')
        let line = match line.find('#') {
            Some(pos) => &line[..pos],
            None => line,
        };
        let line = line.trim();

        // Skip empty lines
        if line.is_empty() {
            continue;
        }

        // Parse "KEY VALUE [MODIFIERS...]" format
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        let key = parts[0];

        // Handle include directive
        if key == "include" {
            if parts.len() != 2 {
                return Err(format!(
                    "{}:{}: Invalid include syntax, expected 'include <path>'",
                    path.display(),
                    line_num
                ));
            }

            let include_path = if let Some(parent) = path.parent() {
                parent.join(parts[1])
            } else {
                PathBuf::from(parts[1])
            };

            // Recursively parse included file
            let included_config = parse_hwdef_with_includes(&include_path, include_chain)?;

            // Merge included configuration
            if platform.is_none() {
                platform = Some(included_config.platform);
            }
            if motor_count.is_none() && included_config.motor_count > 0 {
                motor_count = Some(included_config.motor_count);
            }
            if servo_count.is_none() && included_config.servo_count > 0 {
                servo_count = Some(included_config.servo_count);
            }
            if esc_count.is_none() && included_config.esc_count > 0 {
                esc_count = Some(included_config.esc_count);
            }
            if stepper_count.is_none() && included_config.stepper_count > 0 {
                stepper_count = Some(included_config.stepper_count);
            }

            // Merge pins from included file (can be overridden later)
            for (i, motor) in included_config.motor_pins.iter().enumerate() {
                let motor_num = i + 1;
                pins.entry(format!("M{}_IN1", motor_num))
                    .or_insert(motor.in1);
                pins.entry(format!("M{}_IN2", motor_num))
                    .or_insert(motor.in2);
            }
            for (i, servo) in included_config.servo_pins.iter().enumerate() {
                pins.entry(format!("SERVO{}_PWM", i + 1))
                    .or_insert(servo.pwm);
            }
            for (i, esc) in included_config.esc_pins.iter().enumerate() {
                pins.entry(format!("ESC{}_PWM", i + 1)).or_insert(esc.pwm);
            }
            for (i, stepper) in included_config.stepper_pins.iter().enumerate() {
                let stepper_num = i + 1;
                pins.entry(format!("STEPPER{}_STEP", stepper_num))
                    .or_insert(stepper.step);
                pins.entry(format!("STEPPER{}_DIR", stepper_num))
                    .or_insert(stepper.dir);
                pins.entry(format!("STEPPER{}_EN", stepper_num))
                    .or_insert(stepper.enable);
                if let Some(ms1) = stepper.ms1 {
                    pins.entry(format!("STEPPER{}_MS1", stepper_num))
                        .or_insert(ms1);
                }
            }
            if let Some(buzzer) = included_config.buzzer {
                pins.entry("BUZZER".to_string()).or_insert(buzzer);
            }
            if let Some(led) = included_config.led {
                pins.entry("LED_WS2812".to_string()).or_insert(led);
            }
            if let Some(battery_adc) = included_config.battery_adc {
                pins.entry("BATTERY_ADC".to_string()).or_insert(battery_adc);
            }

            // Merge all named pins from included file (for macro generation)
            for (name, pin) in included_config.all_pins {
                pins.entry(name).or_insert(pin);
            }

            continue;
        }

        // Handle undef directive
        if key == "undef" {
            if parts.len() != 2 {
                return Err(format!(
                    "{}:{}: Invalid undef syntax, expected 'undef <pin_name>'",
                    path.display(),
                    line_num
                ));
            }

            let pin_name = parts[1];
            if pins.remove(pin_name).is_none() {
                println!(
                    "cargo:warning={}:{}: undef of non-existent pin '{}' (ignored)",
                    path.display(),
                    line_num,
                    pin_name
                );
            }
            continue;
        }

        // Ensure we have at least a value after the key
        if parts.len() < 2 {
            return Err(format!(
                "{}:{}: Invalid syntax, expected 'KEY VALUE [MODIFIERS...]', got '{}'",
                path.display(),
                line_num,
                line
            ));
        }

        let value_and_modifiers = &parts[1..];

        match key {
            "PLATFORM" => {
                if value_and_modifiers.is_empty() {
                    return Err(format!(
                        "{}:{}: Missing value for PLATFORM",
                        path.display(),
                        line_num
                    ));
                }
                platform = Some(value_and_modifiers[0].to_string());
            }
            "MOTOR_COUNT" => {
                if value_and_modifiers.is_empty() {
                    return Err(format!(
                        "{}:{}: Missing value for MOTOR_COUNT",
                        path.display(),
                        line_num
                    ));
                }
                motor_count = Some(value_and_modifiers[0].parse::<usize>().map_err(|_| {
                    format!(
                        "{}:{}: Invalid motor count '{}', expected number 0-8",
                        path.display(),
                        line_num,
                        value_and_modifiers[0]
                    )
                })?);
            }
            "SERVO_COUNT" => {
                if value_and_modifiers.is_empty() {
                    return Err(format!(
                        "{}:{}: Missing value for SERVO_COUNT",
                        path.display(),
                        line_num
                    ));
                }
                servo_count = Some(value_and_modifiers[0].parse::<usize>().map_err(|_| {
                    format!(
                        "{}:{}: Invalid servo count '{}', expected number 0-8",
                        path.display(),
                        line_num,
                        value_and_modifiers[0]
                    )
                })?);
            }
            "ESC_COUNT" => {
                if value_and_modifiers.is_empty() {
                    return Err(format!(
                        "{}:{}: Missing value for ESC_COUNT",
                        path.display(),
                        line_num
                    ));
                }
                esc_count = Some(value_and_modifiers[0].parse::<usize>().map_err(|_| {
                    format!(
                        "{}:{}: Invalid ESC count '{}', expected number 0-8",
                        path.display(),
                        line_num,
                        value_and_modifiers[0]
                    )
                })?);
            }
            "STEPPER_COUNT" => {
                if value_and_modifiers.is_empty() {
                    return Err(format!(
                        "{}:{}: Missing value for STEPPER_COUNT",
                        path.display(),
                        line_num
                    ));
                }
                stepper_count = Some(value_and_modifiers[0].parse::<usize>().map_err(|_| {
                    format!(
                        "{}:{}: Invalid stepper count '{}', expected number 0-8",
                        path.display(),
                        line_num,
                        value_and_modifiers[0]
                    )
                })?);
            }
            key if key.starts_with("M") && (key.ends_with("_IN1") || key.ends_with("_IN2")) => {
                // Motor pins: M1_IN1, M1_IN2, M2_IN1, M2_IN2, etc.
                let pin_config = parse_pin_config(path, line_num, key, value_and_modifiers)?;
                pins.insert(key.to_string(), pin_config);
            }
            key if key.starts_with("SERVO") && key.ends_with("_PWM") => {
                // Servo pins: SERVO1_PWM, SERVO2_PWM, etc.
                let pin_config = parse_pin_config(path, line_num, key, value_and_modifiers)?;
                pins.insert(key.to_string(), pin_config);
            }
            key if key.starts_with("ESC") && key.ends_with("_PWM") => {
                // ESC pins: ESC1_PWM, ESC2_PWM, etc.
                let pin_config = parse_pin_config(path, line_num, key, value_and_modifiers)?;
                pins.insert(key.to_string(), pin_config);
            }
            key if key.starts_with("STEPPER")
                && (key.ends_with("_STEP")
                    || key.ends_with("_DIR")
                    || key.ends_with("_EN")
                    || key.ends_with("_MS1")) =>
            {
                // Stepper pins: STEPPER1_STEP, STEPPER1_DIR, STEPPER1_EN, STEPPER1_MS1, etc.
                let pin_config = parse_pin_config(path, line_num, key, value_and_modifiers)?;
                pins.insert(key.to_string(), pin_config);
            }
            "BUZZER" | "LED_WS2812" | "BATTERY_ADC" | "BNO086_RST" | "BNO086_INT"
            | "BNO086_SDA" | "BNO086_SCL" | "UART0_TX" | "UART0_RX" | "WIFI_PWR" | "WIFI_DIO"
            | "WIFI_CS" | "WIFI_CLK" => {
                let pin_config = parse_pin_config(path, line_num, key, value_and_modifiers)?;
                pins.insert(key.to_string(), pin_config);
            }
            _ => {
                return Err(format!(
                    "{}:{}: Unknown key '{}'",
                    path.display(),
                    line_num,
                    key
                ));
            }
        }
    }

    // Validate required fields
    let platform = platform
        .ok_or_else(|| format!("{}: Missing required PLATFORM directive", path.display()))?;

    // Extract actuator counts (default to 0 if not specified)
    let motor_count = motor_count.unwrap_or(0);
    let servo_count = servo_count.unwrap_or(0);
    let esc_count = esc_count.unwrap_or(0);
    let stepper_count = stepper_count.unwrap_or(0);

    // Validate counts
    if motor_count > 8 {
        return Err(format!(
            "{}: MOTOR_COUNT must be 0-8, got {}",
            path.display(),
            motor_count
        ));
    }
    if servo_count > 8 {
        return Err(format!(
            "{}: SERVO_COUNT must be 0-8, got {}",
            path.display(),
            servo_count
        ));
    }
    if esc_count > 8 {
        return Err(format!(
            "{}: ESC_COUNT must be 0-8, got {}",
            path.display(),
            esc_count
        ));
    }
    if stepper_count > 8 {
        return Err(format!(
            "{}: STEPPER_COUNT must be 0-8, got {}",
            path.display(),
            stepper_count
        ));
    }

    // Note: We don't check if at least one actuator is configured here,
    // because this function is also used to parse included hwdef files
    // (like boards/common/rp2350.hwdef) which may not define actuators.
    // The final validation is done in validate_hwdef() in main().

    // Extract motor pins
    let mut motor_pins = Vec::new();
    for motor_num in 1..=motor_count {
        let in1_key = format!("M{}_IN1", motor_num);
        let in2_key = format!("M{}_IN2", motor_num);

        let in1 = pins.get(&in1_key).copied().ok_or_else(|| {
            format!(
                "{}: Missing required pin {} for motor {}",
                path.display(),
                in1_key,
                motor_num
            )
        })?;

        let in2 = pins.get(&in2_key).copied().ok_or_else(|| {
            format!(
                "{}: Missing required pin {} for motor {}",
                path.display(),
                in2_key,
                motor_num
            )
        })?;

        motor_pins.push(MotorPinPair { in1, in2 });
    }

    // Extract servo pins
    let mut servo_pins = Vec::new();
    for servo_num in 1..=servo_count {
        let pwm_key = format!("SERVO{}_PWM", servo_num);

        let pwm = pins.get(&pwm_key).copied().ok_or_else(|| {
            format!(
                "{}: Missing required pin {} for servo {}",
                path.display(),
                pwm_key,
                servo_num
            )
        })?;

        servo_pins.push(ServoPins { pwm });
    }

    // Extract ESC pins
    let mut esc_pins = Vec::new();
    for esc_num in 1..=esc_count {
        let pwm_key = format!("ESC{}_PWM", esc_num);

        let pwm = pins.get(&pwm_key).copied().ok_or_else(|| {
            format!(
                "{}: Missing required pin {} for ESC {}",
                path.display(),
                pwm_key,
                esc_num
            )
        })?;

        esc_pins.push(EscPins { pwm });
    }

    // Extract stepper pins
    let mut stepper_pins = Vec::new();
    for stepper_num in 1..=stepper_count {
        let step_key = format!("STEPPER{}_STEP", stepper_num);
        let dir_key = format!("STEPPER{}_DIR", stepper_num);
        let en_key = format!("STEPPER{}_EN", stepper_num);
        let ms1_key = format!("STEPPER{}_MS1", stepper_num);

        let step = pins.get(&step_key).copied().ok_or_else(|| {
            format!(
                "{}: Missing required pin {} for stepper {}",
                path.display(),
                step_key,
                stepper_num
            )
        })?;

        let dir = pins.get(&dir_key).copied().ok_or_else(|| {
            format!(
                "{}: Missing required pin {} for stepper {}",
                path.display(),
                dir_key,
                stepper_num
            )
        })?;

        let enable = pins.get(&en_key).copied().ok_or_else(|| {
            format!(
                "{}: Missing required pin {} for stepper {}",
                path.display(),
                en_key,
                stepper_num
            )
        })?;

        let ms1 = pins.get(&ms1_key).copied();

        stepper_pins.push(StepperPins {
            step,
            dir,
            enable,
            ms1,
        });
    }

    Ok(HwDefConfig {
        platform,
        motor_count,
        motor_pins,
        servo_count,
        servo_pins,
        esc_count,
        esc_pins,
        stepper_count,
        stepper_pins,
        buzzer: pins.get("BUZZER").copied(),
        led: pins.get("LED_WS2812").copied(),
        battery_adc: pins.get("BATTERY_ADC").copied(),
        all_pins: pins,
    })
}

/// Check if GPIO is a reserved pin on RP2350
fn check_reserved_pins_rp2350(gpio: u8, name: &str, path: &Path, line_num: Option<usize>) {
    let warning = if (0..=1).contains(&gpio) {
        Some(format!(
            "GPIO {} ({}) is reserved for UART0 (default serial console) on RP2350",
            gpio, name
        ))
    } else if (47..=53).contains(&gpio) {
        Some(format!(
            "GPIO {} ({}) is reserved for QSPI flash on RP2350",
            gpio, name
        ))
    } else {
        None
    };

    if let Some(warning_msg) = warning {
        if let Some(ln) = line_num {
            println!(
                "cargo:warning={}:{}: WARNING: {}",
                path.display(),
                ln,
                warning_msg
            );
        } else {
            println!("cargo:warning={}: WARNING: {}", path.display(), warning_msg);
        }
    }
}

/// Validate GPIO range for platform
fn validate_gpio_range(
    path: &Path,
    gpio: u8,
    name: &str,
    platform: &str,
    min: u8,
    max: u8,
) -> Result<(), String> {
    if gpio < min || gpio > max {
        return Err(format!(
            "{}: GPIO {} invalid for {} (platform {}, valid range: {}-{})",
            path.display(),
            gpio,
            name,
            platform,
            min,
            max
        ));
    }
    Ok(())
}

/// Validate hwdef configuration
fn validate_hwdef(config: &HwDefConfig, path: &Path) -> Result<(), String> {
    // Check that at least one actuator type is configured
    if config.motor_count == 0
        && config.servo_count == 0
        && config.esc_count == 0
        && config.stepper_count == 0
    {
        return Err(format!(
            "{}: At least one actuator type must be configured (MOTOR_COUNT, SERVO_COUNT, ESC_COUNT, or STEPPER_COUNT > 0)",
            path.display()
        ));
    }

    // Platform-specific GPIO range validation
    let (min_gpio, max_gpio) = match config.platform.as_str() {
        "rp2350" => (0u8, 29u8),
        _ => {
            return Err(format!(
                "{}: Unsupported platform '{}'",
                path.display(),
                config.platform
            ))
        }
    };

    let mut pin_names: HashMap<u8, Vec<String>> = HashMap::new();

    // Validate and collect motor pins
    for (i, motor) in config.motor_pins.iter().enumerate() {
        let motor_num = i + 1;
        let in1_name = format!("M{}_IN1", motor_num);
        let in2_name = format!("M{}_IN2", motor_num);

        validate_gpio_range(
            path,
            motor.in1.gpio,
            &in1_name,
            &config.platform,
            min_gpio,
            max_gpio,
        )?;
        validate_gpio_range(
            path,
            motor.in2.gpio,
            &in2_name,
            &config.platform,
            min_gpio,
            max_gpio,
        )?;

        // Check for reserved pins on RP2350
        if config.platform == "rp2350" {
            check_reserved_pins_rp2350(motor.in1.gpio, &in1_name, path, None);
            check_reserved_pins_rp2350(motor.in2.gpio, &in2_name, path, None);
        }

        pin_names
            .entry(motor.in1.gpio)
            .or_default()
            .push(in1_name.clone());
        pin_names
            .entry(motor.in2.gpio)
            .or_default()
            .push(in2_name.clone());
    }

    // Validate and collect servo pins
    for (i, servo) in config.servo_pins.iter().enumerate() {
        let servo_num = i + 1;
        let pwm_name = format!("SERVO{}_PWM", servo_num);

        validate_gpio_range(
            path,
            servo.pwm.gpio,
            &pwm_name,
            &config.platform,
            min_gpio,
            max_gpio,
        )?;

        if config.platform == "rp2350" {
            check_reserved_pins_rp2350(servo.pwm.gpio, &pwm_name, path, None);
        }

        pin_names.entry(servo.pwm.gpio).or_default().push(pwm_name);
    }

    // Validate and collect ESC pins
    for (i, esc) in config.esc_pins.iter().enumerate() {
        let esc_num = i + 1;
        let pwm_name = format!("ESC{}_PWM", esc_num);

        validate_gpio_range(
            path,
            esc.pwm.gpio,
            &pwm_name,
            &config.platform,
            min_gpio,
            max_gpio,
        )?;

        if config.platform == "rp2350" {
            check_reserved_pins_rp2350(esc.pwm.gpio, &pwm_name, path, None);
        }

        pin_names.entry(esc.pwm.gpio).or_default().push(pwm_name);
    }

    // Validate and collect stepper pins
    for (i, stepper) in config.stepper_pins.iter().enumerate() {
        let stepper_num = i + 1;
        let step_name = format!("STEPPER{}_STEP", stepper_num);
        let dir_name = format!("STEPPER{}_DIR", stepper_num);
        let en_name = format!("STEPPER{}_EN", stepper_num);

        validate_gpio_range(
            path,
            stepper.step.gpio,
            &step_name,
            &config.platform,
            min_gpio,
            max_gpio,
        )?;
        validate_gpio_range(
            path,
            stepper.dir.gpio,
            &dir_name,
            &config.platform,
            min_gpio,
            max_gpio,
        )?;
        validate_gpio_range(
            path,
            stepper.enable.gpio,
            &en_name,
            &config.platform,
            min_gpio,
            max_gpio,
        )?;

        if config.platform == "rp2350" {
            check_reserved_pins_rp2350(stepper.step.gpio, &step_name, path, None);
            check_reserved_pins_rp2350(stepper.dir.gpio, &dir_name, path, None);
            check_reserved_pins_rp2350(stepper.enable.gpio, &en_name, path, None);
        }

        pin_names
            .entry(stepper.step.gpio)
            .or_default()
            .push(step_name.clone());
        pin_names
            .entry(stepper.dir.gpio)
            .or_default()
            .push(dir_name.clone());
        pin_names
            .entry(stepper.enable.gpio)
            .or_default()
            .push(en_name.clone());

        if let Some(ms1) = stepper.ms1 {
            let ms1_name = format!("STEPPER{}_MS1", stepper_num);
            validate_gpio_range(
                path,
                ms1.gpio,
                &ms1_name,
                &config.platform,
                min_gpio,
                max_gpio,
            )?;

            if config.platform == "rp2350" {
                check_reserved_pins_rp2350(ms1.gpio, &ms1_name, path, None);
            }

            pin_names.entry(ms1.gpio).or_default().push(ms1_name);
        }
    }

    // Validate and collect optional peripheral pins
    let optional_pins = [
        (config.buzzer, "BUZZER"),
        (config.led, "LED_WS2812"),
        (config.battery_adc, "BATTERY_ADC"),
    ];

    for (pin_opt, name) in optional_pins {
        if let Some(pin) = pin_opt {
            validate_gpio_range(path, pin.gpio, name, &config.platform, min_gpio, max_gpio)?;

            if config.platform == "rp2350" {
                check_reserved_pins_rp2350(pin.gpio, name, path, None);
            }

            pin_names
                .entry(pin.gpio)
                .or_default()
                .push(name.to_string());
        }
    }

    // Check for duplicates
    for (gpio, names) in pin_names {
        if names.len() > 1 {
            return Err(format!(
                "{}: GPIO {} used multiple times: {}",
                path.display(),
                gpio,
                names.join(", ")
            ));
        }
    }

    Ok(())
}

/// Generate Rust code from hwdef configuration
fn generate_board_config(config: &HwDefConfig) -> String {
    let mut output = String::new();

    // File header
    output.push_str("// Auto-generated from hwdef.dat - DO NOT EDIT\n");
    output.push_str("// Generated at build time by build.rs\n");
    output.push_str("// This file is included in board.rs, so types are already in scope\n\n");

    // Helper function to generate PinConfig
    let gen_pin_config = |pin: &PinConfig| -> String {
        format!(
            "PinConfig {{ gpio: {}, pin_type: PinType::{:?}, pull: PullMode::{:?}, output_mode: OutputMode::{:?}, speed: Speed::{:?} }}",
            pin.gpio, pin.pin_type, pin.pull, pin.output_mode, pin.speed
        )
    };

    // Generate BOARD_CONFIG constant
    output.push_str("pub const BOARD_CONFIG: BoardPinConfig = BoardPinConfig {\n");

    // Generate motors array
    output.push_str("    motors: [\n");
    for (i, motor) in config.motor_pins.iter().enumerate() {
        output.push_str("        MotorPins {\n");
        output.push_str(&format!(
            "            in1: {},\n",
            gen_pin_config(&motor.in1)
        ));
        output.push_str(&format!(
            "            in2: {},\n",
            gen_pin_config(&motor.in2)
        ));
        output.push_str("        }");
        if i < config.motor_pins.len() - 1 {
            output.push_str(",\n");
        } else {
            output.push('\n');
        }
    }

    // Pad to 4 motors if fewer defined
    for i in config.motor_pins.len()..4 {
        let dummy_pin = PinConfig::new(255u8); // Invalid GPIO to catch accidental use
        output.push_str("        MotorPins {\n");
        output.push_str(&format!(
            "            in1: {},\n",
            gen_pin_config(&dummy_pin)
        ));
        output.push_str(&format!(
            "            in2: {},\n",
            gen_pin_config(&dummy_pin)
        ));
        output.push_str("        }");
        if i < 3 {
            output.push_str(",\n");
        } else {
            output.push('\n');
        }
    }
    output.push_str("    ],\n");

    // Generate optional peripherals
    if let Some(buzzer) = config.buzzer {
        output.push_str(&format!("    buzzer: Some({}),\n", gen_pin_config(&buzzer)));
    } else {
        output.push_str("    buzzer: None,\n");
    }

    if let Some(led) = config.led {
        output.push_str(&format!("    led: Some({}),\n", gen_pin_config(&led)));
    } else {
        output.push_str("    led: None,\n");
    }

    if let Some(battery_adc) = config.battery_adc {
        output.push_str(&format!(
            "    battery_adc: Some({}),\n",
            gen_pin_config(&battery_adc)
        ));
    } else {
        output.push_str("    battery_adc: None,\n");
    }

    output.push_str("};\n");

    output
}

/// Generate compile-time pin macros from hwdef configuration
///
/// Produces `board_pins.rs` with macros that map hwdef pin names to
/// embassy-rp monomorphic pin types at compile time.
fn generate_board_pins(config: &HwDefConfig) -> String {
    let mut output = String::new();

    output.push_str("// Auto-generated from hwdef configuration - DO NOT EDIT\n");
    output.push_str("// Generated at build time by build.rs\n");
    output.push_str("// Provides compile-time pin mapping macros for embassy-rp\n\n");

    // board_pin!($p, NAME) - maps hwdef name to $p.PIN_X
    output.push_str("/// Maps a hwdef pin name to the corresponding `$p.PIN_X` peripheral.\n");
    output.push_str("macro_rules! board_pin {\n");
    let mut sorted_pins: Vec<_> = config.all_pins.iter().collect();
    sorted_pins.sort_by_key(|(name, _)| (*name).clone());
    for (name, pin) in &sorted_pins {
        output.push_str(&format!(
            "    ($p:expr, {}) => {{ $p.PIN_{} }};\n",
            name, pin.gpio
        ));
    }
    output.push_str("}\n\n");

    // board_steal_pin!(NAME) - maps hwdef name to PIN_X::steal()
    output.push_str(
        "/// Maps a hwdef pin name to `embassy_rp::peripherals::PIN_X::steal()` for bus recovery.\n",
    );
    output.push_str("macro_rules! board_steal_pin {\n");
    for (name, pin) in &sorted_pins {
        output.push_str(&format!(
            "    ({}) => {{ hal::peripherals::PIN_{}::steal() }};\n",
            name, pin.gpio
        ));
    }
    output.push_str("}\n\n");

    // Motor-specific macros (only if motors are configured)
    if config.motor_count > 0 {
        // board_pwm_slice!($p, N) - maps motor number to PWM slice
        // PWM slice = (min_gpio / 2) % 8
        output.push_str(
            "/// Maps a motor number (1-based) to the corresponding `$p.PWM_SLICEX` peripheral.\n",
        );
        output.push_str("macro_rules! board_pwm_slice {\n");
        for (i, motor) in config.motor_pins.iter().enumerate() {
            let motor_num = i + 1;
            let min_gpio = motor.in1.gpio.min(motor.in2.gpio);
            let slice = (min_gpio / 2) % 8;
            output.push_str(&format!(
                "    ($p:expr, {}) => {{ $p.PWM_SLICE{} }};\n",
                motor_num, slice
            ));
        }
        output.push_str("}\n\n");

        // board_motor_pin_a!($p, N) - even GPIO (PWM channel A)
        output.push_str(
            "/// Maps a motor number to the even GPIO pin (PWM channel A) for `Pwm::new_output_ab`.\n",
        );
        output.push_str("macro_rules! board_motor_pin_a {\n");
        for (i, motor) in config.motor_pins.iter().enumerate() {
            let motor_num = i + 1;
            let even_gpio = if motor.in1.gpio % 2 == 0 {
                motor.in1.gpio
            } else {
                motor.in2.gpio
            };
            output.push_str(&format!(
                "    ($p:expr, {}) => {{ $p.PIN_{} }};\n",
                motor_num, even_gpio
            ));
        }
        output.push_str("}\n\n");

        // board_motor_pin_b!($p, N) - odd GPIO (PWM channel B)
        output.push_str(
            "/// Maps a motor number to the odd GPIO pin (PWM channel B) for `Pwm::new_output_ab`.\n",
        );
        output.push_str("macro_rules! board_motor_pin_b {\n");
        for (i, motor) in config.motor_pins.iter().enumerate() {
            let motor_num = i + 1;
            let odd_gpio = if motor.in1.gpio % 2 == 1 {
                motor.in1.gpio
            } else {
                motor.in2.gpio
            };
            output.push_str(&format!(
                "    ($p:expr, {}) => {{ $p.PIN_{} }};\n",
                motor_num, odd_gpio
            ));
        }
        output.push_str("}\n\n");
    }

    output
}

fn main() {
    // Generate BUILD_ID for debugging (changes on every build)
    use std::time::{SystemTime, UNIX_EPOCH};
    let build_id = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs();
    println!("cargo:rustc-env=BUILD_ID={}", build_id);

    // Read WiFi configuration from environment variables (optional)
    // These are used as default values when parameter storage is empty

    // WiFi SSID (network name)
    if let Ok(ssid) = env::var("NET_SSID") {
        println!("cargo:rustc-env=NET_SSID={}", ssid);
        println!("cargo:warning=Using NET_SSID from environment: {}", ssid);
    } else {
        println!("cargo:rustc-env=NET_SSID=");
    }

    // WiFi password
    if let Ok(password) = env::var("NET_PASS") {
        println!("cargo:rustc-env=NET_PASS={}", password);
        println!("cargo:warning=Using NET_PASS from environment (hidden)");
    } else {
        println!("cargo:rustc-env=NET_PASS=");
    }

    // DHCP enabled (default: true)
    if let Ok(dhcp) = env::var("NET_DHCP") {
        println!("cargo:rustc-env=NET_DHCP={}", dhcp);
        println!("cargo:warning=Using NET_DHCP from environment: {}", dhcp);
    } else {
        println!("cargo:rustc-env=NET_DHCP=true");
    }

    // Static IP (used if DHCP=false)
    if let Ok(ip) = env::var("NET_IP") {
        println!("cargo:rustc-env=NET_IP={}", ip);
        println!("cargo:warning=Using NET_IP from environment: {}", ip);
    } else {
        println!("cargo:rustc-env=NET_IP=0.0.0.0");
    }

    // Network mask
    if let Ok(netmask) = env::var("NET_NETMASK") {
        println!("cargo:rustc-env=NET_NETMASK={}", netmask);
        println!(
            "cargo:warning=Using NET_NETMASK from environment: {}",
            netmask
        );
    } else {
        println!("cargo:rustc-env=NET_NETMASK=255.255.255.0");
    }

    // Gateway
    if let Ok(gateway) = env::var("NET_GATEWAY") {
        println!("cargo:rustc-env=NET_GATEWAY={}", gateway);
        println!(
            "cargo:warning=Using NET_GATEWAY from environment: {}",
            gateway
        );
    } else {
        println!("cargo:rustc-env=NET_GATEWAY=0.0.0.0");
    }

    // Rerun if environment variables change
    println!("cargo:rerun-if-env-changed=NET_SSID");
    println!("cargo:rerun-if-env-changed=NET_PASS");
    println!("cargo:rerun-if-env-changed=NET_DHCP");
    println!("cargo:rerun-if-env-changed=NET_IP");
    println!("cargo:rerun-if-env-changed=NET_NETMASK");
    println!("cargo:rerun-if-env-changed=NET_GATEWAY");

    // GPS Privacy Offset (for hiding actual location in public displays)
    // Values are parsed as f32 and added to GPS coordinates
    let lat_offset: f32 = env::var("GPS_LAT_OFFSET")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    println!("cargo:rustc-env=GPS_LAT_OFFSET={}", lat_offset);
    if lat_offset != 0.0 {
        println!("cargo:warning=Using GPS_LAT_OFFSET: {}", lat_offset);
    }

    let lon_offset: f32 = env::var("GPS_LON_OFFSET")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.0);
    println!("cargo:rustc-env=GPS_LON_OFFSET={}", lon_offset);
    if lon_offset != 0.0 {
        println!("cargo:warning=Using GPS_LON_OFFSET: {}", lon_offset);
    }

    println!("cargo:rerun-if-env-changed=GPS_LAT_OFFSET");
    println!("cargo:rerun-if-env-changed=GPS_LON_OFFSET");

    // Process hwdef files for board pin configuration
    // boards directory is at workspace root (../../boards from crates/firmware)
    let board_name = env::var("BOARD").unwrap_or_else(|_| "freenove_standard".to_string());
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR not set");
    let workspace_root = PathBuf::from(&manifest_dir)
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf();
    let hwdef_path = workspace_root
        .join("boards")
        .join(format!("{}.hwdef", board_name));

    println!("cargo:rerun-if-changed={}", hwdef_path.display());
    println!("cargo:rerun-if-env-changed=BOARD");

    // Parse and validate hwdef
    let hwdef_config = match parse_hwdef(&hwdef_path) {
        Ok(config) => config,
        Err(e) => {
            panic!("Failed to parse hwdef file: {}", e);
        }
    };

    if let Err(e) = validate_hwdef(&hwdef_config, &hwdef_path) {
        panic!("hwdef validation failed: {}", e);
    }

    // Generate Rust code
    let generated_code = generate_board_config(&hwdef_config);
    let generated_pins = generate_board_pins(&hwdef_config);

    // Write to OUT_DIR
    let out_dir = env::var("OUT_DIR").expect("OUT_DIR not set");
    let out_dir_path = PathBuf::from(&out_dir);

    let config_path = out_dir_path.join("board_config.rs");
    let mut file = fs::File::create(&config_path).expect("Failed to create board_config.rs");
    file.write_all(generated_code.as_bytes())
        .expect("Failed to write board_config.rs");

    let pins_path = out_dir_path.join("board_pins.rs");
    let mut file = fs::File::create(&pins_path).expect("Failed to create board_pins.rs");
    file.write_all(generated_pins.as_bytes())
        .expect("Failed to write board_pins.rs");

    println!(
        "cargo:warning=Generated board configuration for '{}' from {}",
        board_name,
        hwdef_path.display()
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn create_temp_hwdef(content: &str) -> std::io::Result<tempfile::NamedTempFile> {
        let mut file = tempfile::NamedTempFile::new()?;
        file.write_all(content.as_bytes())?;
        file.flush()?;
        Ok(file)
    }

    #[test]
    fn test_parse_valid_hwdef() {
        let content = r#"
# Test hwdef file
PLATFORM rp2350
MOTOR_COUNT 4

# Motor pins
M1_IN1 18
M1_IN2 19
M2_IN1 20
M2_IN2 21
M3_IN1 6
M3_IN2 7
M4_IN1 8
M4_IN2 9

# Optional peripherals
BUZZER 2
LED_WS2812 16
BATTERY_ADC 26
"#;
        let file = create_temp_hwdef(content).unwrap();
        let config = parse_hwdef(file.path()).unwrap();

        assert_eq!(config.platform, "rp2350");
        assert_eq!(config.motor_count, 4);
        assert_eq!(config.motor_pins.len(), 4);
        assert_eq!(config.motor_pins[0].in1.gpio, 18);
        assert_eq!(config.motor_pins[0].in2.gpio, 19);
        assert_eq!(config.motor_pins[1].in1.gpio, 20);
        assert_eq!(config.motor_pins[1].in2.gpio, 21);
        assert_eq!(config.buzzer.map(|p| p.gpio), Some(2));
        assert_eq!(config.led.map(|p| p.gpio), Some(16));
        assert_eq!(config.battery_adc.map(|p| p.gpio), Some(26));
    }

    #[test]
    fn test_parse_minimal_hwdef() {
        let content = r#"
PLATFORM rp2350
MOTOR_COUNT 2
M1_IN1 18
M1_IN2 19
M2_IN1 20
M2_IN2 21
"#;
        let file = create_temp_hwdef(content).unwrap();
        let config = parse_hwdef(file.path()).unwrap();

        assert_eq!(config.platform, "rp2350");
        assert_eq!(config.motor_count, 2);
        assert_eq!(config.motor_pins.len(), 2);
        assert_eq!(config.servo_count, 0);
        assert_eq!(config.esc_count, 0);
        assert_eq!(config.stepper_count, 0);
        assert_eq!(config.buzzer, None);
        assert_eq!(config.led, None);
        assert_eq!(config.battery_adc, None);
    }

    #[test]
    fn test_parse_missing_platform() {
        let content = r#"
MOTOR_COUNT 2
M1_IN1 18
M1_IN2 19
M2_IN1 20
M2_IN2 21
"#;
        let file = create_temp_hwdef(content).unwrap();
        let result = parse_hwdef(file.path());

        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Missing required PLATFORM"));
    }

    #[test]
    fn test_parse_missing_actuators() {
        let content = r#"
PLATFORM rp2350
"#;
        let file = create_temp_hwdef(content).unwrap();
        let result = parse_hwdef(file.path());

        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .contains("At least one actuator type must be configured"));
    }

    #[test]
    fn test_parse_missing_motor_pin() {
        let content = r#"
PLATFORM rp2350
MOTOR_COUNT 4
M1_IN1 18
M1_IN2 19
M2_IN1 20
M2_IN2 21
M3_IN1 6
M3_IN2 7
M4_IN1 8
"#;
        let file = create_temp_hwdef(content).unwrap();
        let result = parse_hwdef(file.path());

        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Missing required pin M4_IN2"));
    }

    #[test]
    fn test_parse_invalid_syntax() {
        let content = r#"
PLATFORM rp2350
M1_IN1 18 extra
"#;
        let file = create_temp_hwdef(content).unwrap();
        let result = parse_hwdef(file.path());

        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Invalid syntax"));
    }

    #[test]
    fn test_parse_invalid_gpio_number() {
        let content = r#"
PLATFORM rp2350
M1_IN1 abc
"#;
        let file = create_temp_hwdef(content).unwrap();
        let result = parse_hwdef(file.path());

        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Invalid GPIO number"));
    }

    #[test]
    fn test_validate_valid_config() {
        let config = HwDefConfig {
            platform: "rp2350".to_string(),
            motor_count: 4,
            motor_pins: vec![
                MotorPinPair {
                    in1: PinConfig::new(18),
                    in2: PinConfig::new(19),
                },
                MotorPinPair {
                    in1: PinConfig::new(20),
                    in2: PinConfig::new(21),
                },
                MotorPinPair {
                    in1: PinConfig::new(6),
                    in2: PinConfig::new(7),
                },
                MotorPinPair {
                    in1: PinConfig::new(8),
                    in2: PinConfig::new(9),
                },
            ],
            servo_count: 0,
            servo_pins: vec![],
            esc_count: 0,
            esc_pins: vec![],
            stepper_count: 0,
            stepper_pins: vec![],
            buzzer: Some(PinConfig::new(2)),
            led: Some(PinConfig::new(16)),
            battery_adc: Some(PinConfig::new(26)),
            all_pins: HashMap::new(),
        };

        let file = create_temp_hwdef("").unwrap();
        let result = validate_hwdef(&config, file.path());
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_duplicate_pins() {
        let config = HwDefConfig {
            platform: "rp2350".to_string(),
            motor_count: 4,
            motor_pins: vec![
                MotorPinPair {
                    in1: PinConfig::new(18),
                    in2: PinConfig::new(19),
                },
                MotorPinPair {
                    in1: PinConfig::new(18),
                    in2: PinConfig::new(21),
                }, // Duplicate GPIO 18
                MotorPinPair {
                    in1: PinConfig::new(6),
                    in2: PinConfig::new(7),
                },
                MotorPinPair {
                    in1: PinConfig::new(8),
                    in2: PinConfig::new(9),
                },
            ],
            servo_count: 0,
            servo_pins: vec![],
            esc_count: 0,
            esc_pins: vec![],
            stepper_count: 0,
            stepper_pins: vec![],
            buzzer: None,
            led: None,
            battery_adc: None,
            all_pins: HashMap::new(),
        };

        let file = create_temp_hwdef("").unwrap();
        let result = validate_hwdef(&config, file.path());
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("GPIO 18 used multiple times"));
    }

    #[test]
    fn test_validate_invalid_gpio_range() {
        let config = HwDefConfig {
            platform: "rp2350".to_string(),
            motor_count: 4,
            motor_pins: vec![
                MotorPinPair {
                    in1: PinConfig::new(35),
                    in2: PinConfig::new(19),
                }, // GPIO 35 out of range
                MotorPinPair {
                    in1: PinConfig::new(20),
                    in2: PinConfig::new(21),
                },
                MotorPinPair {
                    in1: PinConfig::new(6),
                    in2: PinConfig::new(7),
                },
                MotorPinPair {
                    in1: PinConfig::new(8),
                    in2: PinConfig::new(9),
                },
            ],
            servo_count: 0,
            servo_pins: vec![],
            esc_count: 0,
            esc_pins: vec![],
            stepper_count: 0,
            stepper_pins: vec![],
            buzzer: None,
            led: None,
            battery_adc: None,
            all_pins: HashMap::new(),
        };

        let file = create_temp_hwdef("").unwrap();
        let result = validate_hwdef(&config, file.path());
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("GPIO 35 invalid"));
    }

    #[test]
    fn test_validate_unsupported_platform() {
        let config = HwDefConfig {
            platform: "esp32".to_string(),
            motor_count: 4,
            motor_pins: vec![
                MotorPinPair {
                    in1: PinConfig::new(18),
                    in2: PinConfig::new(19),
                },
                MotorPinPair {
                    in1: PinConfig::new(20),
                    in2: PinConfig::new(21),
                },
                MotorPinPair {
                    in1: PinConfig::new(6),
                    in2: PinConfig::new(7),
                },
                MotorPinPair {
                    in1: PinConfig::new(8),
                    in2: PinConfig::new(9),
                },
            ],
            servo_count: 0,
            servo_pins: vec![],
            esc_count: 0,
            esc_pins: vec![],
            stepper_count: 0,
            stepper_pins: vec![],
            buzzer: None,
            led: None,
            battery_adc: None,
            all_pins: HashMap::new(),
        };

        let file = create_temp_hwdef("").unwrap();
        let result = validate_hwdef(&config, file.path());
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Unsupported platform"));
    }

    #[test]
    fn test_parse_servo_config() {
        let content = r#"
PLATFORM rp2350
SERVO_COUNT 2
SERVO1_PWM 10
SERVO2_PWM 11
"#;
        let file = create_temp_hwdef(content).unwrap();
        let config = parse_hwdef(file.path()).unwrap();

        assert_eq!(config.platform, "rp2350");
        assert_eq!(config.servo_count, 2);
        assert_eq!(config.servo_pins.len(), 2);
        assert_eq!(config.servo_pins[0].pwm.gpio, 10);
        assert_eq!(config.servo_pins[1].pwm.gpio, 11);
    }

    #[test]
    fn test_parse_mixed_actuators() {
        let content = r#"
PLATFORM rp2350
MOTOR_COUNT 2
M1_IN1 18
M1_IN2 19
M2_IN1 20
M2_IN2 21
SERVO_COUNT 1
SERVO1_PWM 10
ESC_COUNT 1
ESC1_PWM 12
"#;
        let file = create_temp_hwdef(content).unwrap();
        let config = parse_hwdef(file.path()).unwrap();

        assert_eq!(config.motor_count, 2);
        assert_eq!(config.servo_count, 1);
        assert_eq!(config.esc_count, 1);
        assert_eq!(config.stepper_count, 0);
    }
}
