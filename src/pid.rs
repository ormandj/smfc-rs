/// PID controller with anti-windup, rate limiting, and derivative EMA filtering.
///
/// Designed for thermal fan control: output is a duty cycle percentage (0-100).
/// The controller converges to the minimum duty needed to hold the temperature
/// at the setpoint.
#[derive(Debug)]
pub struct PidController {
    setpoint: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    min_output: f64,
    max_output: f64,
    max_rate: f64,
    d_filter: f64,

    // State
    integral: f64,
    prev_error: Option<f64>,
    filtered_derivative: f64,
    prev_output: f64,
}

pub struct PidParams {
    pub setpoint: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub min_output: f64,
    pub max_output: f64,
    pub max_rate: f64,
    pub d_filter: f64,
}

impl PidController {
    pub fn new(params: PidParams) -> Self {
        Self {
            setpoint: params.setpoint,
            kp: params.kp,
            ki: params.ki,
            kd: params.kd,
            min_output: params.min_output,
            max_output: params.max_output,
            max_rate: params.max_rate,
            d_filter: params.d_filter,
            integral: 0.0,
            prev_error: None,
            filtered_derivative: 0.0,
            prev_output: params.min_output,
        }
    }

    /// Update the PID controller with a new temperature reading.
    ///
    /// `dt` is the time since the last update in seconds.
    /// Returns the computed duty cycle percentage, clamped and rate-limited.
    pub fn update(&mut self, current_temp: f64, dt: f64) -> f64 {
        if dt <= 0.0 {
            return self.prev_output;
        }

        let error = current_temp - self.setpoint;

        // Proportional
        let p_term = self.kp * error;

        // Integral (accumulated before clamping, undone if saturated)
        self.integral += error * dt;
        // Clamp integral so its contribution can never exceed the actuator range.
        // Bounds windup even when output never hits max_output exactly.
        if self.ki.abs() > f64::EPSILON {
            let i_limit = (self.max_output - self.min_output) / self.ki.abs();
            self.integral = self.integral.clamp(-i_limit, i_limit);
        }
        // Leaky integrator inside a deadband: when the process sits at the
        // setpoint (error ≈ 0), slowly bleed the integral toward zero so a
        // past transient can't hold the actuator high indefinitely. The P and
        // D terms are zero here, so any steady-state duty is coming from the
        // integral alone — if it's excessive, temperature will rise and the
        // integral will rebuild to the correct level.
        const INTEGRAL_DEADBAND: f64 = 0.5; // °C
        const INTEGRAL_LEAK_PER_SEC: f64 = 0.005; // ~140s half-life at setpoint
        if error.abs() < INTEGRAL_DEADBAND {
            let decay = (1.0 - INTEGRAL_LEAK_PER_SEC * dt).max(0.0);
            self.integral *= decay;
        }
        let i_term = self.ki * self.integral;

        // Derivative with EMA filtering
        let raw_derivative = match self.prev_error {
            Some(prev) => (error - prev) / dt,
            None => 0.0,
        };
        self.filtered_derivative =
            self.d_filter * raw_derivative + (1.0 - self.d_filter) * self.filtered_derivative;
        let d_term = self.kd * self.filtered_derivative;

        self.prev_error = Some(error);

        // Combined output
        let mut output = p_term + i_term + d_term;

        // Clamp output
        output = output.clamp(self.min_output, self.max_output);

        // Anti-windup: if output is saturated, undo the last integral accumulation
        // to prevent the integral from winding up further in the saturated direction
        if (output >= self.max_output && error > 0.0) || (output <= self.min_output && error < 0.0)
        {
            self.integral -= error * dt;
        }

        // Rate limiting: smooth acoustic transitions
        let delta = output - self.prev_output;
        if delta.abs() > self.max_rate {
            output = self.prev_output + delta.signum() * self.max_rate;
        }

        self.prev_output = output;
        output
    }

    pub fn output(&self) -> f64 {
        self.prev_output
    }

    pub fn setpoint(&self) -> f64 {
        self.setpoint
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_params() -> PidParams {
        PidParams {
            setpoint: 60.0,
            kp: 2.0,
            ki: 0.1,
            kd: 1.0,
            min_output: 25.0,
            max_output: 100.0,
            max_rate: 50.0, // high rate limit so it doesn't interfere with basic tests
            d_filter: 0.3,
        }
    }

    #[test]
    fn below_setpoint_settles_at_min() {
        let mut pid = PidController::new(default_params());
        // Temp well below setpoint — output should converge to min
        for _ in 0..100 {
            pid.update(40.0, 1.0);
        }
        assert!(
            (pid.output() - 25.0).abs() < 0.1,
            "expected ~25.0, got {}",
            pid.output()
        );
    }

    #[test]
    fn above_setpoint_increases_output() {
        let mut pid = PidController::new(PidParams {
            kp: 5.0, // higher gain so P term exceeds min_output
            ..default_params()
        });
        let out1 = pid.update(70.0, 1.0);
        let out2 = pid.update(70.0, 1.0);
        // Both should be above min since temp > setpoint and Kp is high enough
        assert!(out1 > 25.0, "expected > 25.0, got {out1}");
        assert!(out2 > 25.0, "expected > 25.0, got {out2}");
        // Integral accumulates, so second output should be >= first
        assert!(out2 >= out1, "expected {out2} >= {out1}");
    }

    #[test]
    fn anti_windup_prevents_integral_blowup() {
        let mut pid = PidController::new(default_params());
        // Push output to max for many cycles
        for _ in 0..200 {
            pid.update(95.0, 1.0);
        }
        assert!(
            (pid.output() - 100.0).abs() < 0.1,
            "expected ~100.0, got {}",
            pid.output()
        );

        // Now drop temp below setpoint — should recover reasonably fast
        // (if integral wound up, it would take forever to come down)
        for _ in 0..50 {
            pid.update(40.0, 1.0);
        }
        assert!(
            pid.output() < 50.0,
            "expected < 50.0 after cooldown, got {}",
            pid.output()
        );
    }

    #[test]
    fn integral_does_not_wind_up_below_saturation() {
        // Reproduces the zone1 HDD bug: error briefly positive, then pinned at
        // setpoint (error=0) forever. Output must not be held high by a wound-up
        // integral when P and D terms are both ~0.
        let mut pid = PidController::new(PidParams {
            max_rate: 200.0, // don't let rate limiting mask the effect
            ..default_params()
        });
        // Wind up while above setpoint (setpoint=60, temp=68)
        for _ in 0..50 {
            pid.update(68.0, 30.0);
        }
        // Now pinned at setpoint forever
        for _ in 0..200 {
            pid.update(60.0, 30.0);
        }
        // With error=0 and a bounded integral, output should decay toward min.
        // Without the clamp, the wound-up integral would hold it near max.
        assert!(
            pid.output() < 40.0,
            "integral wound up: output={}",
            pid.output()
        );
    }

    #[test]
    fn rate_limiting_smooths_transitions() {
        let mut pid = PidController::new(PidParams {
            max_rate: 3.0,
            ..default_params()
        });

        // Start at min, then sudden hot temp
        let out1 = pid.update(40.0, 1.0); // should be at/near min
        let out2 = pid.update(90.0, 1.0); // wants to jump high

        let delta = (out2 - out1).abs();
        assert!(
            delta <= 3.0 + 0.01,
            "rate limit violated: delta={delta}, max_rate=3.0"
        );
    }

    #[test]
    fn derivative_responds_to_rapid_change() {
        let mut pid = PidController::new(PidParams {
            kp: 5.0,         // high proportional to exceed min_output
            kd: 10.0,        // high derivative gain
            ki: 0.0,         // disable integral for isolation
            min_output: 0.0, // low floor to see the full PID response
            ..default_params()
        });

        // Steady at setpoint
        pid.update(60.0, 1.0);
        let steady = pid.update(60.0, 1.0);

        // Sudden 10°C spike
        let spike = pid.update(70.0, 1.0);

        // With Kp=5, P=50. With Kd=10 and filtered derivative, D adds significantly more.
        // spike should be notably higher than steady (which should be ~0 at setpoint)
        assert!(
            spike > steady + 15.0,
            "derivative should amplify response: steady={steady}, spike={spike}"
        );
    }

    #[test]
    fn output_clamped_to_bounds() {
        let mut pid = PidController::new(PidParams {
            max_rate: 200.0, // don't let rate limiting interfere
            ..default_params()
        });

        // Very hot — should not exceed max
        for _ in 0..50 {
            pid.update(200.0, 1.0);
        }
        assert!(
            pid.output() <= 100.0,
            "output exceeded max: {}",
            pid.output()
        );

        // Very cold — should not go below min
        for _ in 0..50 {
            pid.update(-20.0, 1.0);
        }
        assert!(pid.output() >= 25.0, "output below min: {}", pid.output());
    }

    #[test]
    fn zero_dt_returns_previous_output() {
        let mut pid = PidController::new(default_params());
        let out1 = pid.update(70.0, 1.0);
        let out2 = pid.update(80.0, 0.0);
        assert!(
            (out1 - out2).abs() < f64::EPSILON,
            "zero dt should return previous output"
        );
    }
}
