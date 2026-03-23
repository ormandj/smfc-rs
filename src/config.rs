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
    pub hwmon_driver: String,
    pub zone: ZoneId,
    #[serde(default = "default_poll_interval")]
    pub poll_interval_secs: u64,
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub temp_selector: TempSelector,
    pub pid: PidConfig,
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

#[derive(Debug, Deserialize)]
pub struct PidConfig {
    /// Target temperature setpoint (°C)
    pub setpoint: f64,

    /// Warning temperature — instant 100% if exceeded.
    /// Auto-detected from hwmon temp*_max; config value overrides only if lower.
    pub warn_temp: Option<f64>,

    /// Proportional gain
    #[serde(default = "default_kp")]
    pub kp: f64,

    /// Integral gain
    #[serde(default = "default_ki")]
    pub ki: f64,

    /// Derivative gain
    #[serde(default = "default_kd")]
    pub kd: f64,

    /// Minimum PID output (duty %)
    #[serde(default = "default_min_duty")]
    pub min_duty: u8,

    /// Maximum PID output (duty %)
    #[serde(default = "default_max_duty")]
    pub max_duty: u8,

    /// Maximum duty change per poll cycle (acoustic smoothing)
    #[serde(default = "default_max_rate")]
    pub max_rate: f64,

    /// Derivative EMA filter coefficient (0-1, lower = smoother)
    #[serde(default = "default_d_filter")]
    pub d_filter: f64,
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

fn default_kp() -> f64 {
    2.0
}

fn default_ki() -> f64 {
    0.1
}

fn default_kd() -> f64 {
    1.0
}

fn default_max_rate() -> f64 {
    5.0
}

fn default_d_filter() -> f64 {
    0.3
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
        if c.pid.d_filter < 0.0 || c.pid.d_filter > 1.0 {
            return Err(crate::error::Error::Config(format!(
                "controller '{}': d_filter must be in [0.0, 1.0]",
                c.name
            )));
        }
    }

    Ok(config)
}
