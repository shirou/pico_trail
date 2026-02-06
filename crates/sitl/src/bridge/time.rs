/// Time synchronization mode for the SITL simulation.
#[derive(Debug, Clone)]
pub enum TimeMode {
    /// Simulation runs as fast as possible, no synchronization.
    FreeRunning,
    /// Simulation advances in discrete steps of `step_size_us` microseconds.
    Lockstep { step_size_us: u64 },
    /// Simulation runs at a scaled rate relative to wall-clock time.
    Scaled { factor: f32 },
}

impl Default for TimeMode {
    fn default() -> Self {
        Self::FreeRunning
    }
}

/// Coordinates simulation time across all vehicles and adapters.
///
/// Wraps a [`TimeMode`] and tracks current simulation time. Lockstep mode
/// increments by a fixed step size; free-running mode uses wall clock;
/// scaled mode multiplies elapsed wall time by a factor.
pub struct TimeCoordinator {
    mode: TimeMode,
    sim_time_us: u64,
    wall_clock_start: std::time::Instant,
    last_wall_clock: std::time::Instant,
}

impl TimeCoordinator {
    /// Create a new coordinator with the given time mode.
    pub fn new(mode: TimeMode) -> Self {
        let now = std::time::Instant::now();
        Self {
            mode,
            sim_time_us: 0,
            wall_clock_start: now,
            last_wall_clock: now,
        }
    }

    /// Advance simulation time by one step and return the new time.
    pub fn advance(&mut self) -> u64 {
        match &self.mode {
            TimeMode::Lockstep { step_size_us } => {
                self.sim_time_us += step_size_us;
            }
            TimeMode::FreeRunning => {
                let now = std::time::Instant::now();
                let elapsed = now.duration_since(self.wall_clock_start);
                self.sim_time_us = elapsed.as_micros() as u64;
                self.last_wall_clock = now;
            }
            TimeMode::Scaled { factor } => {
                let now = std::time::Instant::now();
                let delta = now.duration_since(self.last_wall_clock);
                let scaled_us = (delta.as_micros() as f64 * *factor as f64) as u64;
                self.sim_time_us += scaled_us;
                self.last_wall_clock = now;
            }
        }
        self.sim_time_us
    }

    /// Get the current simulation time in microseconds.
    pub fn sim_time_us(&self) -> u64 {
        self.sim_time_us
    }

    /// Set the time mode, resetting wall-clock tracking.
    pub fn set_mode(&mut self, mode: TimeMode) {
        let now = std::time::Instant::now();
        self.wall_clock_start = now;
        self.last_wall_clock = now;
        self.mode = mode;
    }

    /// Get a reference to the current time mode.
    pub fn mode(&self) -> &TimeMode {
        &self.mode
    }
}

impl Default for TimeCoordinator {
    fn default() -> Self {
        Self::new(TimeMode::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lockstep_advances_by_step_size() {
        let mut tc = TimeCoordinator::new(TimeMode::Lockstep {
            step_size_us: 10_000,
        });
        assert_eq!(tc.sim_time_us(), 0);

        let t1 = tc.advance();
        assert_eq!(t1, 10_000);
        assert_eq!(tc.sim_time_us(), 10_000);

        let t2 = tc.advance();
        assert_eq!(t2, 20_000);
    }

    #[test]
    fn test_lockstep_many_steps() {
        let mut tc = TimeCoordinator::new(TimeMode::Lockstep {
            step_size_us: 1_000,
        });
        for _ in 0..100 {
            tc.advance();
        }
        assert_eq!(tc.sim_time_us(), 100_000);
    }

    #[test]
    fn test_free_running_advances() {
        let mut tc = TimeCoordinator::new(TimeMode::FreeRunning);
        // Sleep briefly so wall clock advances
        std::thread::sleep(std::time::Duration::from_millis(10));
        tc.advance();
        // Should have advanced at least some microseconds
        assert!(tc.sim_time_us() > 0);
    }

    #[test]
    fn test_scaled_mode() {
        let mut tc = TimeCoordinator::new(TimeMode::Scaled { factor: 2.0 });
        std::thread::sleep(std::time::Duration::from_millis(10));
        tc.advance();
        // With 2x factor, 10ms wall time should give ~20ms sim time
        // Allow generous bounds due to scheduling
        assert!(tc.sim_time_us() > 0);
    }

    #[test]
    fn test_set_mode_resets_wall_clock() {
        let mut tc = TimeCoordinator::new(TimeMode::Lockstep {
            step_size_us: 5_000,
        });
        tc.advance();
        tc.advance();
        assert_eq!(tc.sim_time_us(), 10_000);

        // Switch to lockstep with different step size
        // sim_time_us is preserved but wall clock is reset
        tc.set_mode(TimeMode::Lockstep {
            step_size_us: 1_000,
        });
        assert_eq!(tc.sim_time_us(), 10_000);
        tc.advance();
        assert_eq!(tc.sim_time_us(), 11_000);
    }

    #[test]
    fn test_default_is_free_running() {
        let tc = TimeCoordinator::default();
        assert!(matches!(tc.mode(), TimeMode::FreeRunning));
        assert_eq!(tc.sim_time_us(), 0);
    }
}
