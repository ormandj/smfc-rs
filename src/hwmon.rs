use std::path::{Path, PathBuf};

use tracing::{debug, warn};

use crate::error::{Error, Result};

/// A discovered hwmon temperature sensor.
#[derive(Debug, Clone)]
pub struct HwmonSensor {
    /// Sensor label from temp*_label (e.g., "Tctl"), if available
    pub label: Option<String>,
    /// Path to temp*_input file (millidegrees C)
    pub temp_path: PathBuf,
    /// Hardware-reported maximum temperature from temp*_max (°C), if available
    pub max_temp: Option<f64>,
}

impl HwmonSensor {
    /// Read the current temperature in degrees Celsius.
    pub fn read_temp(&self) -> Result<f64> {
        let raw = std::fs::read_to_string(&self.temp_path).map_err(|e| {
            Error::Hwmon(format!("failed to read {}: {e}", self.temp_path.display()))
        })?;
        let millidegrees: i64 = raw.trim().parse().map_err(|e| {
            Error::Hwmon(format!("failed to parse {}: {e}", self.temp_path.display()))
        })?;
        Ok(millidegrees as f64 / 1000.0)
    }
}

/// Read a millidegree value from a hwmon file and convert to degrees C.
fn read_millidegree_file(path: &Path) -> Option<f64> {
    let raw = std::fs::read_to_string(path).ok()?;
    let millidegrees: i64 = raw.trim().parse().ok()?;
    Some(millidegrees as f64 / 1000.0)
}

/// Discover all hwmon temperature sensors matching a given driver name.
///
/// Walks `/sys/class/hwmon/hwmon*/name` and returns sensors for each matching
/// hwmon device. For each device, discovers all `temp*_input` files along with
/// their labels and hardware-reported thresholds.
pub fn discover_sensors(driver_name: &str) -> Result<Vec<HwmonSensor>> {
    discover_sensors_in("/sys/class/hwmon", driver_name)
}

fn discover_sensors_in(base: &str, driver_name: &str) -> Result<Vec<HwmonSensor>> {
    let mut sensors = Vec::new();
    let base_path = Path::new(base);

    let entries = std::fs::read_dir(base_path)
        .map_err(|e| Error::Hwmon(format!("failed to read {base}: {e}")))?;

    for entry in entries {
        let entry = entry.map_err(|e| Error::Hwmon(format!("readdir error: {e}")))?;
        let hwmon_dir = entry.path();
        let name_file = hwmon_dir.join("name");

        let name = match std::fs::read_to_string(&name_file) {
            Ok(n) => n.trim().to_string(),
            Err(_) => continue,
        };

        if name != driver_name {
            continue;
        }

        debug!(hwmon = %hwmon_dir.display(), driver = driver_name, "found matching hwmon device");

        // Find all temp*_input files
        let dir_entries = match std::fs::read_dir(&hwmon_dir) {
            Ok(e) => e,
            Err(_) => continue,
        };

        for temp_entry in dir_entries {
            let temp_entry = match temp_entry {
                Ok(e) => e,
                Err(_) => continue,
            };
            let filename = temp_entry.file_name();
            let filename_str = filename.to_string_lossy();

            if !filename_str.starts_with("temp") || !filename_str.ends_with("_input") {
                continue;
            }

            let temp_path = temp_entry.path();

            // Extract the temp index prefix (e.g., "temp1" from "temp1_input")
            let prefix = filename_str.trim_end_matches("_input");

            // Read optional label
            let label_path = hwmon_dir.join(format!("{prefix}_label"));
            let label = std::fs::read_to_string(&label_path)
                .ok()
                .map(|s| s.trim().to_string());

            // Read optional max/crit thresholds
            let max_path = hwmon_dir.join(format!("{prefix}_max"));
            let max_temp = read_millidegree_file(&max_path);

            let crit_path = hwmon_dir.join(format!("{prefix}_crit"));
            let crit_temp = read_millidegree_file(&crit_path);

            debug!(
                path = %temp_path.display(),
                label = label.as_deref().unwrap_or("none"),
                max_temp = max_temp.map(|t| format!("{t:.0}°C")).unwrap_or_else(|| "none".into()).as_str(),
                crit_temp = crit_temp.map(|t| format!("{t:.0}°C")).unwrap_or_else(|| "none".into()).as_str(),
                "discovered sensor"
            );

            sensors.push(HwmonSensor {
                label,
                temp_path,
                max_temp,
            });
        }
    }

    if sensors.is_empty() {
        warn!(driver = driver_name, "no hwmon sensors found");
    }

    Ok(sensors)
}

/// Determine the effective warn_temp for a set of sensors.
///
/// Uses the minimum `temp*_max` across all sensors (most conservative device limit).
/// If a config override is provided, uses the lower of the two.
/// Returns None if neither source provides a value.
pub fn effective_warn_temp(sensors: &[HwmonSensor], config_override: Option<f64>) -> Option<f64> {
    let hw_max = sensors.iter().filter_map(|s| s.max_temp).reduce(f64::min);

    match (hw_max, config_override) {
        (Some(hw), Some(cfg)) => Some(hw.min(cfg)),
        (Some(hw), None) => Some(hw),
        (None, Some(cfg)) => Some(cfg),
        (None, None) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sensor(max_temp: Option<f64>) -> HwmonSensor {
        HwmonSensor {
            label: None,
            temp_path: PathBuf::from("/nonexistent"),
            max_temp,
        }
    }

    #[test]
    fn warn_temp_from_hardware() {
        let sensors = vec![sensor(Some(85.0)), sensor(Some(90.0))];
        assert_eq!(effective_warn_temp(&sensors, None), Some(85.0));
    }

    #[test]
    fn warn_temp_config_lower_wins() {
        let sensors = vec![sensor(Some(85.0))];
        assert_eq!(effective_warn_temp(&sensors, Some(80.0)), Some(80.0));
    }

    #[test]
    fn warn_temp_hardware_lower_wins() {
        let sensors = vec![sensor(Some(70.0))];
        assert_eq!(effective_warn_temp(&sensors, Some(80.0)), Some(70.0));
    }

    #[test]
    fn warn_temp_config_only() {
        let sensors = vec![sensor(None)];
        assert_eq!(effective_warn_temp(&sensors, Some(75.0)), Some(75.0));
    }

    #[test]
    fn warn_temp_none() {
        let sensors = vec![sensor(None)];
        assert_eq!(effective_warn_temp(&sensors, None), None);
    }
}
