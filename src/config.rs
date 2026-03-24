use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct AppConfig {
    pub general: GeneralConfig,
    #[serde(rename = "controllers")]
    pub controllers: Vec<ControllerConfig>,
}

#[derive(Debug, Deserialize)]
pub struct GeneralConfig {
    /// Path to IPMI device (default: /dev/ipmi0)
    #[serde(default = "default_ipmi_device")]
    pub ipmi_device: String,

    /// Log level (default: info)
    #[serde(default = "default_log_level")]
    pub log_level: String,

    /// Minimum duty % for Zone 0 (compute chamber)
    #[serde(default = "default_min_duty")]
    pub min_duty_zone0: u8,

    /// Minimum duty % for Zone 1 (storage chamber)
    #[serde(default = "default_min_duty")]
    pub min_duty_zone1: u8,
}

#[derive(Debug, Deserialize)]
pub struct ControllerConfig {
    pub name: String,
    /// hwmon driver name (e.g., "k10temp", "nvme"). Mutually exclusive with ipmi_sensors.
    pub hwmon_driver: Option<String>,
    /// IPMI sensor IDs to read via "Get Sensor Reading" command. Mutually exclusive with hwmon_driver.
    #[serde(default)]
    pub ipmi_sensors: Vec<IpmiSensorConfig>,
    pub zone: ZoneId,
    #[serde(default = "default_poll_interval")]
    pub poll_interval_secs: u64,
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub temp_selector: TempSelector,
    pub pid: PidConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct IpmiSensorConfig {
    /// IPMI sensor number (e.g., 0x74 for GPU5)
    pub id: u8,
    /// Human-readable label for logging
    pub label: String,
    /// Upper non-critical threshold from BMC SDR (used as warn_temp)
    pub warn_temp: Option<f64>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "lowercase")]
pub enum ZoneId {
    Zone0,
    Zone1,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub enum TempSelector {
    /// Use the maximum temperature across all sensors
    #[default]
    Max,
    /// Use a specific sensor label (e.g., "Tctl")
    Specific(String),
}

/// Thermal mass of the device — determines how quickly it responds to airflow changes.
/// Controls integral accumulation rate and derivative responsiveness.
#[derive(Debug, Clone, Copy, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum ThermalMass {
    /// Bare chip, no heatsink (NVMe, small ICs) — responds in seconds
    Low,
    /// Heatsink with moderate mass (CPU cooler, small passive sinks) — responds in 10-30s
    #[default]
    Medium,
    /// Large passive heatsink or metal body (ConnectX-6, HDDs) — responds in minutes
    High,
}

#[derive(Debug, Deserialize)]
pub struct PidConfig {
    /// Target temperature setpoint (°C)
    pub setpoint: f64,

    /// Warning temperature — instant 100% if exceeded.
    /// Auto-detected from hwmon temp*_max; config value overrides only if lower.
    pub warn_temp: Option<f64>,

    /// Thermal mass of the device (low/medium/high). Determines PID gain scaling.
    /// Default: medium
    #[serde(default)]
    pub thermal_mass: ThermalMass,

    /// Whether the device has its own active cooling (fan on heatsink).
    /// When true, case fans are supplementary — PID gains are reduced.
    /// Default: false
    #[serde(default)]
    pub active_cooling: bool,

    /// Explicit PID gains — if set, override the auto-derived values from thermal_mass.
    pub kp: Option<f64>,
    pub ki: Option<f64>,
    pub kd: Option<f64>,

    /// Minimum PID output (duty %)
    #[serde(default = "default_min_duty")]
    pub min_duty: u8,

    /// Maximum PID output (duty %)
    #[serde(default = "default_max_duty")]
    pub max_duty: u8,

    /// Maximum duty change per poll cycle (acoustic smoothing).
    /// Auto-derived from thermal_mass if not set.
    pub max_rate: Option<f64>,

    /// Derivative EMA filter coefficient (0-1, lower = smoother).
    /// Auto-derived from thermal_mass if not set.
    pub d_filter: Option<f64>,
}

/// Resolved PID parameters after applying thermal_mass and active_cooling defaults.
#[derive(Debug, Clone)]
pub struct ResolvedPidParams {
    pub setpoint: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub min_duty: u8,
    pub max_duty: u8,
    pub max_rate: f64,
    pub d_filter: f64,
}

impl PidConfig {
    /// Resolve PID gains from thermal_mass + active_cooling, with explicit overrides.
    pub fn resolve(&self) -> ResolvedPidParams {
        // Base gains derived from thermal mass
        let (base_kp, base_ki, base_kd, base_max_rate, base_d_filter) = match self.thermal_mass {
            ThermalMass::Low => (3.0, 0.15, 0.5, 8.0, 0.4),
            ThermalMass::Medium => (2.0, 0.08, 1.0, 5.0, 0.3),
            ThermalMass::High => (1.0, 0.02, 2.0, 3.0, 0.15),
        };

        // Active cooling reduces gains — case fans are supplementary but still
        // needed for ambient air. 0.7x keeps reasonable responsiveness.
        let active_scale = if self.active_cooling { 0.7 } else { 1.0 };

        ResolvedPidParams {
            setpoint: self.setpoint,
            kp: self.kp.unwrap_or(base_kp) * active_scale,
            ki: self.ki.unwrap_or(base_ki) * active_scale,
            kd: self.kd.unwrap_or(base_kd) * active_scale,
            min_duty: self.min_duty,
            max_duty: self.max_duty,
            max_rate: self.max_rate.unwrap_or(base_max_rate),
            d_filter: self.d_filter.unwrap_or(base_d_filter),
        }
    }
}

fn default_ipmi_device() -> String {
    "/dev/ipmi0".to_string()
}

fn default_log_level() -> String {
    "info".to_string()
}

fn default_min_duty() -> u8 {
    25
}

fn default_max_duty() -> u8 {
    100
}

fn default_poll_interval() -> u64 {
    5
}

fn default_true() -> bool {
    true
}

pub fn load(path: &str) -> crate::error::Result<AppConfig> {
    let contents = std::fs::read_to_string(path)
        .map_err(|e| crate::error::Error::Config(format!("failed to read {path}: {e}")))?;
    let config: AppConfig = toml::from_str(&contents)?;

    // Validate
    if config.controllers.is_empty() {
        return Err(crate::error::Error::Config(
            "no controllers defined".to_string(),
        ));
    }
    for c in &config.controllers {
        if c.hwmon_driver.is_none() && c.ipmi_sensors.is_empty() {
            return Err(crate::error::Error::Config(format!(
                "controller '{}': must have either hwmon_driver or ipmi_sensors",
                c.name
            )));
        }
        if c.hwmon_driver.is_some() && !c.ipmi_sensors.is_empty() {
            return Err(crate::error::Error::Config(format!(
                "controller '{}': hwmon_driver and ipmi_sensors are mutually exclusive",
                c.name
            )));
        }
        if c.pid.setpoint <= 0.0 {
            return Err(crate::error::Error::Config(format!(
                "controller '{}': setpoint must be positive",
                c.name
            )));
        }
        if c.pid.min_duty > c.pid.max_duty {
            return Err(crate::error::Error::Config(format!(
                "controller '{}': min_duty ({}) > max_duty ({})",
                c.name, c.pid.min_duty, c.pid.max_duty
            )));
        }
        let resolved = c.pid.resolve();
        if resolved.d_filter < 0.0 || resolved.d_filter > 1.0 {
            return Err(crate::error::Error::Config(format!(
                "controller '{}': d_filter must be in [0.0, 1.0]",
                c.name
            )));
        }
    }

    Ok(config)
}
