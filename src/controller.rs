use std::time::{Duration, Instant};

use tracing::{debug, error, warn};

use crate::config::{ControllerConfig, TempSelector, ZoneId};
use crate::error::Result;
use crate::hwmon::HwmonSensor;
use crate::pid::{PidController, PidParams};
use crate::zone::ControllerOutput;

/// A thermal controller that reads sensors, runs a PID loop, and outputs a duty.
pub struct ThermalController {
    pub name: String,
    pub zone: ZoneId,
    sensors: Vec<HwmonSensor>,
    temp_selector: TempSelector,
    pid: PidController,
    warn_temp: f64,
    poll_interval: Duration,
    last_poll: Option<Instant>,
    last_temp: Option<f64>,
}

impl ThermalController {
    pub fn new(config: &ControllerConfig, sensors: Vec<HwmonSensor>, warn_temp: f64) -> Self {
        let resolved = config.pid.resolve();
        let pid = PidController::new(PidParams {
            setpoint: resolved.setpoint,
            kp: resolved.kp,
            ki: resolved.ki,
            kd: resolved.kd,
            min_output: resolved.min_duty as f64,
            max_output: resolved.max_duty as f64,
            max_rate: resolved.max_rate,
            d_filter: resolved.d_filter,
        });

        Self {
            name: config.name.clone(),
            zone: config.zone.clone(),
            sensors,
            temp_selector: config.temp_selector.clone(),
            pid,
            warn_temp,
            poll_interval: Duration::from_secs(config.poll_interval_secs),
            last_poll: None,
            last_temp: None,
        }
    }

    /// Check if it's time to poll, and if so, read sensors and compute duty.
    ///
    /// Returns `None` if it's not yet time to poll.
    /// Returns `Some(output)` with the computed duty and warning status.
    pub fn poll(&mut self) -> Option<ControllerOutput> {
        let now = Instant::now();

        if let Some(last) = self.last_poll
            && now.duration_since(last) < self.poll_interval
        {
            // Not time yet — return the last output if we have one
            return self.last_temp.map(|temp| ControllerOutput {
                zone: self.zone.clone(),
                duty: self.pid.output() as u8,
                temp,
                controller_name: self.name.clone(),
                warning: temp >= self.warn_temp,
            });
        }

        let dt = self
            .last_poll
            .map(|last| now.duration_since(last).as_secs_f64())
            .unwrap_or(self.poll_interval.as_secs_f64());

        self.last_poll = Some(now);

        // Read temperature
        let temp = match self.read_temp() {
            Ok(t) => {
                self.last_temp = Some(t);
                t
            }
            Err(e) => {
                warn!(controller = %self.name, error = %e, "sensor read failed, using last known temp");
                match self.last_temp {
                    Some(t) => t,
                    None => {
                        // No previous reading — assume worst case
                        error!(controller = %self.name, "no previous temp reading, assuming warn_temp");
                        self.warn_temp
                    }
                }
            }
        };

        // Check warn_temp — immediate 100% override
        if temp >= self.warn_temp {
            warn!(
                controller = %self.name,
                temp = format!("{temp:.1}°C"),
                warn_temp = format!("{:.1}°C", self.warn_temp),
                "WARN TEMP EXCEEDED — forcing 100%"
            );
            return Some(ControllerOutput {
                zone: self.zone.clone(),
                duty: 100,
                temp,
                controller_name: self.name.clone(),
                warning: true,
            });
        }

        // Normal PID control
        let duty = self.pid.update(temp, dt);
        let duty_u8 = (duty.round() as u8).min(100);

        debug!(
            controller = %self.name,
            temp = format!("{temp:.1}°C"),
            setpoint = format!("{:.1}°C", self.pid.setpoint()),
            duty = duty_u8,
            "PID output"
        );

        Some(ControllerOutput {
            zone: self.zone.clone(),
            duty: duty_u8,
            temp,
            controller_name: self.name.clone(),
            warning: false,
        })
    }

    fn read_temp(&self) -> Result<f64> {
        match &self.temp_selector {
            TempSelector::Max => {
                let mut max_temp: Option<f64> = None;
                let mut last_err = None;
                for sensor in &self.sensors {
                    match sensor.read_temp() {
                        Ok(t) => {
                            max_temp = Some(max_temp.map_or(t, |cur: f64| cur.max(t)));
                        }
                        Err(e) => {
                            warn!(
                                controller = %self.name,
                                sensor = %sensor.temp_path.display(),
                                error = %e,
                                "individual sensor read failed"
                            );
                            last_err = Some(e);
                        }
                    }
                }
                max_temp.ok_or_else(|| {
                    last_err.unwrap_or_else(|| {
                        crate::error::Error::Hwmon("no sensors available".to_string())
                    })
                })
            }
            TempSelector::Specific(label) => {
                let matching: Vec<_> = self
                    .sensors
                    .iter()
                    .filter(|s| s.label.as_deref() == Some(label.as_str()))
                    .collect();
                if matching.is_empty() {
                    return Err(crate::error::Error::Hwmon(format!(
                        "no sensor with label '{label}' found for {}",
                        self.name
                    )));
                }
                let mut max_temp: Option<f64> = None;
                let mut last_err = None;
                for sensor in &matching {
                    match sensor.read_temp() {
                        Ok(t) => {
                            max_temp = Some(max_temp.map_or(t, |cur: f64| cur.max(t)));
                        }
                        Err(e) => {
                            warn!(
                                controller = %self.name,
                                sensor = %sensor.temp_path.display(),
                                error = %e,
                                "individual sensor read failed"
                            );
                            last_err = Some(e);
                        }
                    }
                }
                max_temp.ok_or_else(|| {
                    last_err.unwrap_or_else(|| {
                        crate::error::Error::Hwmon("no sensors available".to_string())
                    })
                })
            }
        }
    }
}
