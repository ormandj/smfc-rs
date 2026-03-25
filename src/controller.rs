use std::sync::Arc;
use std::time::{Duration, Instant};

use tracing::{debug, error, info, warn};

use crate::config::{ControllerConfig, IpmiSensorConfig, TempSelector, ZoneId};
use crate::error::Result;
use crate::hwmon::HwmonSensor;
use crate::ipmi::IpmiDevice;
use crate::pid::{PidController, PidParams};
use crate::zone::ControllerOutput;

/// A temperature sensor that can be read.
pub enum TempSensor {
    Hwmon(HwmonSensor),
    Ipmi {
        device: Arc<IpmiDevice>,
        sensor_id: u8,
        label: String,
    },
}

impl TempSensor {
    pub fn read_temp(&self) -> Result<f64> {
        match self {
            TempSensor::Hwmon(s) => s.read_temp(),
            TempSensor::Ipmi {
                device,
                sensor_id,
                label: _,
            } => crate::ipmi::read_sensor_temp(device, *sensor_id),
        }
    }

    pub fn label(&self) -> Option<&str> {
        match self {
            TempSensor::Hwmon(s) => s.label.as_deref(),
            TempSensor::Ipmi { label, .. } => Some(label.as_str()),
        }
    }

    pub fn display_name(&self) -> String {
        match self {
            TempSensor::Hwmon(s) => format!("{}", s.temp_path.display()),
            TempSensor::Ipmi {
                sensor_id, label, ..
            } => format!("ipmi:0x{sensor_id:02x}({label})"),
        }
    }
}

/// An IPMI sensor that failed discovery and should be retried.
pub struct PendingIpmiSensor {
    config: IpmiSensorConfig,
    device: Arc<IpmiDevice>,
}

/// A thermal controller that reads sensors, runs a PID loop, and outputs a duty.
pub struct ThermalController {
    pub name: String,
    pub zone: ZoneId,
    sensors: Vec<TempSensor>,
    pending_sensors: Vec<PendingIpmiSensor>,
    last_retry: Option<Instant>,
    temp_selector: TempSelector,
    pid: PidController,
    warn_temp: f64,
    poll_interval: Duration,
    last_poll: Option<Instant>,
    last_temp: Option<f64>,
}

const RETRY_INTERVAL: Duration = Duration::from_secs(30);

impl ThermalController {
    pub fn new(
        config: &ControllerConfig,
        sensors: Vec<TempSensor>,
        pending_sensors: Vec<PendingIpmiSensor>,
        warn_temp: f64,
    ) -> Self {
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
            pending_sensors,
            last_retry: None,
            temp_selector: config.temp_selector.clone(),
            pid,
            warn_temp,
            poll_interval: Duration::from_secs(config.poll_interval_secs),
            last_poll: None,
            last_temp: None,
        }
    }

    /// Create a PendingIpmiSensor from a config + device.
    pub fn make_pending(config: IpmiSensorConfig, device: Arc<IpmiDevice>) -> PendingIpmiSensor {
        PendingIpmiSensor { config, device }
    }

    /// Check if it's time to poll, and if so, read sensors and compute duty.
    pub fn poll(&mut self) -> Option<ControllerOutput> {
        let now = Instant::now();

        // Retry pending IPMI sensors periodically
        self.retry_pending_sensors(now);

        if let Some(last) = self.last_poll
            && now.duration_since(last) < self.poll_interval
        {
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
                        error!(controller = %self.name, "no previous temp reading, assuming warn_temp");
                        self.warn_temp
                    }
                }
            }
        };

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

    fn retry_pending_sensors(&mut self, now: Instant) {
        if self.pending_sensors.is_empty() {
            return;
        }

        if let Some(last) = self.last_retry
            && now.duration_since(last) < RETRY_INTERVAL
        {
            return;
        }
        self.last_retry = Some(now);

        let mut still_pending = Vec::new();
        for pending in self.pending_sensors.drain(..) {
            match crate::ipmi::read_sensor_temp(&pending.device, pending.config.id) {
                Ok(temp) => {
                    info!(
                        controller = %self.name,
                        sensor = format!("0x{:02x}", pending.config.id),
                        label = %pending.config.label,
                        temp = format!("{temp:.0}°C"),
                        "IPMI sensor now available (late discovery)"
                    );
                    // Update warn_temp if this sensor has a lower threshold
                    if let Some(sw) = pending.config.warn_temp {
                        self.warn_temp = self.warn_temp.min(sw);
                    }
                    self.sensors.push(TempSensor::Ipmi {
                        device: Arc::clone(&pending.device),
                        sensor_id: pending.config.id,
                        label: pending.config.label.clone(),
                    });
                }
                Err(_) => {
                    still_pending.push(pending);
                }
            }
        }
        self.pending_sensors = still_pending;
    }

    fn read_temp(&self) -> Result<f64> {
        match &self.temp_selector {
            TempSelector::Max => self.read_max_all(),
            TempSelector::Specific(label) => self.read_max_by_label(label),
        }
    }

    fn read_max_all(&self) -> Result<f64> {
        let mut max_temp: Option<f64> = None;
        let mut last_err = None;
        for sensor in &self.sensors {
            match sensor.read_temp() {
                Ok(t) => max_temp = Some(max_temp.map_or(t, |cur: f64| cur.max(t))),
                Err(e) => {
                    warn!(
                        controller = %self.name,
                        sensor = %sensor.display_name(),
                        error = %e,
                        "individual sensor read failed"
                    );
                    last_err = Some(e);
                }
            }
        }
        max_temp.ok_or_else(|| {
            last_err
                .unwrap_or_else(|| crate::error::Error::Hwmon("no sensors available".to_string()))
        })
    }

    fn read_max_by_label(&self, label: &str) -> Result<f64> {
        let matching: Vec<_> = self
            .sensors
            .iter()
            .filter(|s| s.label() == Some(label))
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
                Ok(t) => max_temp = Some(max_temp.map_or(t, |cur: f64| cur.max(t))),
                Err(e) => {
                    warn!(
                        controller = %self.name,
                        sensor = %sensor.display_name(),
                        error = %e,
                        "individual sensor read failed"
                    );
                    last_err = Some(e);
                }
            }
        }
        max_temp.ok_or_else(|| {
            last_err
                .unwrap_or_else(|| crate::error::Error::Hwmon("no sensors available".to_string()))
        })
    }
}
