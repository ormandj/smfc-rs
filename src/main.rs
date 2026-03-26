mod config;
mod controller;
mod error;
mod hwmon;
mod ipmi;
mod pid;
mod zone;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use clap::Parser;
use tracing::{error, info, warn};

use crate::config::IpmiSensorConfig;
use crate::controller::{TempSensor, ThermalController};
use crate::ipmi::{FanMode, IpmiZone};
use crate::zone::{ControllerOutput, ZoneArbitrator};

#[derive(Parser)]
#[command(name = "smfc-rs")]
#[command(about = "Supermicro fan controller — PID-based multi-source thermal management via IPMI")]
struct Cli {
    /// Path to TOML config file
    #[arg(long, short, default_value = "/config/smfc.toml")]
    config: String,

    /// Dry-run mode: read sensors and log decisions without touching IPMI
    #[arg(long)]
    dry_run: bool,
}

fn main() {
    let cli = Cli::parse();

    let cfg = match config::load(&cli.config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("fatal: failed to load config: {e}");
            std::process::exit(1);
        }
    };

    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(&cfg.general.log_level)),
        )
        .with_target(false)
        .init();

    if cli.dry_run {
        info!("dry-run mode enabled — will not send IPMI commands");
    }

    // Open IPMI device (shared across controllers for IPMI sensor reads + fan control)
    let ipmi_dev: Option<Arc<ipmi::IpmiDevice>> = if !cli.dry_run {
        match ipmi::IpmiDevice::open(&cfg.general.ipmi_device) {
            Ok(dev) => Some(Arc::new(dev)),
            Err(e) => {
                error!(error = %e, "failed to open IPMI device — are ipmi_devintf and ipmi_si kernel modules loaded?");
                std::process::exit(1);
            }
        }
    } else {
        info!(device = %cfg.general.ipmi_device, "skipping IPMI device open (dry-run)");
        None
    };

    // Save current fan mode and switch to Full for manual control
    let saved_mode = if let Some(ref dev) = ipmi_dev {
        match ipmi::get_fan_mode(dev) {
            Ok(mode) => {
                info!(mode = %mode, "saved current BMC fan mode");
                if mode != FanMode::Full {
                    if let Err(e) = ipmi::set_fan_mode(dev, FanMode::Full) {
                        error!(error = %e, "failed to set Full fan mode");
                        std::process::exit(1);
                    }
                    info!("switched BMC to Full fan mode (manual duty control)");
                }
                Some(mode)
            }
            Err(e) => {
                error!(error = %e, "failed to read fan mode");
                std::process::exit(1);
            }
        }
    } else {
        info!("skipping fan mode save/set (dry-run)");
        None
    };

    // Register signal handlers
    let shutdown = Arc::new(AtomicBool::new(false));
    for sig in [signal_hook::consts::SIGTERM, signal_hook::consts::SIGINT] {
        let shutdown = Arc::clone(&shutdown);
        signal_hook::flag::register(sig, shutdown).expect("failed to register signal handler");
    }

    // Build controllers
    let mut controllers: Vec<ThermalController> = Vec::new();
    for ctrl_cfg in &cfg.controllers {
        if !ctrl_cfg.enabled {
            info!(controller = %ctrl_cfg.name, "disabled, skipping");
            continue;
        }

        // Build sensors from either hwmon or IPMI source
        let result = match build_sensors(ctrl_cfg, &ipmi_dev) {
            Some(r) => r,
            None => continue,
        };

        let resolved = ctrl_cfg.pid.resolve();
        let pending_count = result.pending.len();
        info!(
            controller = %ctrl_cfg.name,
            source = if ctrl_cfg.hwmon_driver.is_some() { "hwmon" } else { "ipmi" },
            sensors = result.sensors.len(),
            pending = pending_count,
            zone = ?ctrl_cfg.zone,
            setpoint = format!("{:.0}°C", ctrl_cfg.pid.setpoint),
            warn_temp = format!("{:.0}°C", result.warn_temp),
            thermal_mass = ?ctrl_cfg.pid.thermal_mass,
            active_cooling = ctrl_cfg.pid.active_cooling,
            kp = format!("{:.2}", resolved.kp),
            ki = format!("{:.3}", resolved.ki),
            kd = format!("{:.2}", resolved.kd),
            "initialized"
        );

        let pending_sensors: Vec<_> = result
            .pending
            .into_iter()
            .map(|(cfg, dev)| ThermalController::make_pending(cfg, dev))
            .collect();

        controllers.push(ThermalController::new(
            ctrl_cfg,
            result.sensors,
            pending_sensors,
            result.warn_temp,
        ));
    }

    if controllers.is_empty() {
        error!("no controllers enabled — nothing to do");
        std::process::exit(1);
    }

    let mut arbitrator =
        ZoneArbitrator::new(cfg.general.min_duty_zone0, cfg.general.min_duty_zone1);

    info!(controllers = controllers.len(), "starting control loop");

    let tick = Duration::from_secs(1);
    let mut log_counter: u32 = 0;

    while !shutdown.load(Ordering::Relaxed) {
        thread::sleep(tick);

        let mut outputs: Vec<ControllerOutput> = Vec::new();

        for controller in &mut controllers {
            if let Some(output) = controller.poll() {
                outputs.push(output);
            }
        }

        if outputs.is_empty() {
            continue;
        }

        let any_warning = outputs.iter().any(|o| o.warning);
        if any_warning {
            for output in &outputs {
                if output.warning
                    && let Some(dev) = &ipmi_dev
                {
                    let zone = IpmiZone::from(&output.zone);
                    if let Some(update) = arbitrator.force_zone(output.zone.clone(), 100)
                        && let Err(e) = ipmi::set_fan_duty(dev, zone, update.duty)
                    {
                        error!(zone = ?zone, error = %e, "IPMI set duty failed");
                    }
                }
            }
        }

        if !any_warning {
            let updates = arbitrator.arbitrate(&outputs);
            for update in &updates {
                if let Some(ref dev) = ipmi_dev {
                    let zone = IpmiZone::from(&update.zone);
                    if let Err(e) = ipmi::set_fan_duty(dev, zone, update.duty) {
                        error!(zone = ?zone, error = %e, "IPMI set duty failed");
                    }
                }
            }

            log_counter += 1;
            if log_counter >= 30 || !updates.is_empty() {
                log_counter = 0;
                log_status(
                    &outputs,
                    cfg.general.min_duty_zone0,
                    cfg.general.min_duty_zone1,
                );
            }
        }
    }

    info!("shutdown requested");
    if let (Some(dev), Some(mode)) = (&ipmi_dev, saved_mode) {
        info!(mode = %mode, "restoring BMC fan mode");
        if let Err(e) = ipmi::set_fan_mode(dev, mode) {
            error!(error = %e, "failed to restore fan mode");
        } else {
            info!(mode = %mode, "fan mode restored — BMC resumes automatic management");
        }
    }
}

/// Sensor discovery result.
struct SensorResult {
    sensors: Vec<TempSensor>,
    pending: Vec<(IpmiSensorConfig, Arc<ipmi::IpmiDevice>)>,
    warn_temp: f64,
}

/// Build sensors and determine warn_temp for a controller config.
/// Returns None if the controller should be disabled (no sensors found).
fn build_sensors(
    ctrl_cfg: &config::ControllerConfig,
    ipmi_dev: &Option<Arc<ipmi::IpmiDevice>>,
) -> Option<SensorResult> {
    if let Some(ref driver) = ctrl_cfg.hwmon_driver {
        // hwmon-based controller
        let hwmon_sensors = match hwmon::discover_sensors(driver) {
            Ok(s) => s,
            Err(e) => {
                warn!(controller = %ctrl_cfg.name, error = %e, "sensor discovery failed, disabling");
                return None;
            }
        };

        if hwmon_sensors.is_empty() {
            warn!(controller = %ctrl_cfg.name, driver = %driver, "no sensors found, disabling");
            return None;
        }

        // Filter sensors by label for warn_temp if using Specific selector
        let warn_sensors: Vec<_> = match &ctrl_cfg.temp_selector {
            config::TempSelector::Specific(label) => hwmon_sensors
                .iter()
                .filter(|s| s.label.as_deref() == Some(label.as_str()))
                .cloned()
                .collect(),
            config::TempSelector::Max => hwmon_sensors.clone(),
        };
        let warn_temp = match hwmon::effective_warn_temp(&warn_sensors, ctrl_cfg.pid.warn_temp) {
            Some(t) => t,
            None => {
                error!(
                    controller = %ctrl_cfg.name,
                    "no warn_temp available (not in hwmon and not in config)"
                );
                std::process::exit(1);
            }
        };

        let sensors = hwmon_sensors.into_iter().map(TempSensor::Hwmon).collect();
        Some(SensorResult {
            sensors,
            pending: Vec::new(),
            warn_temp,
        })
    } else if !ctrl_cfg.ipmi_sensors.is_empty() {
        // IPMI-based controller
        let dev = match ipmi_dev {
            Some(dev) => Arc::clone(dev),
            None => {
                warn!(controller = %ctrl_cfg.name, "IPMI sensors configured but no IPMI device (dry-run?), disabling");
                return None;
            }
        };

        let mut sensors = Vec::new();
        let mut pending = Vec::new();
        let mut warn_temp: Option<f64> = ctrl_cfg.pid.warn_temp;

        for sensor_cfg in &ctrl_cfg.ipmi_sensors {
            match ipmi::read_sensor_temp(&dev, sensor_cfg.id) {
                Ok(temp) => {
                    info!(
                        controller = %ctrl_cfg.name,
                        sensor = format!("0x{:02x}", sensor_cfg.id),
                        label = %sensor_cfg.label,
                        temp = format!("{temp:.0}°C"),
                        "IPMI sensor found"
                    );
                    if let Some(sw) = sensor_cfg.warn_temp {
                        warn_temp = Some(warn_temp.map_or(sw, |w: f64| w.min(sw)));
                    }
                    sensors.push(TempSensor::Ipmi {
                        device: Arc::clone(&dev),
                        sensor_id: sensor_cfg.id,
                        label: sensor_cfg.label.clone(),
                    });
                }
                Err(e) => {
                    warn!(
                        controller = %ctrl_cfg.name,
                        sensor = format!("0x{:02x}", sensor_cfg.id),
                        label = %sensor_cfg.label,
                        error = %e,
                        "IPMI sensor not available, will retry periodically"
                    );
                    pending.push((sensor_cfg.clone(), Arc::clone(&dev)));
                }
            }
        }

        if sensors.is_empty() && pending.is_empty() {
            warn!(controller = %ctrl_cfg.name, "no IPMI sensors available, disabling");
            return None;
        }

        // If no sensors found yet but some are pending, use the config warn_temp
        // (pending sensors will update warn_temp when they come online)
        let warn_temp = match warn_temp {
            Some(t) => t,
            None if !sensors.is_empty() => {
                error!(
                    controller = %ctrl_cfg.name,
                    "no warn_temp available for IPMI controller (must be in config)"
                );
                std::process::exit(1);
            }
            // All sensors pending — use a safe default from config or first sensor config
            None => ctrl_cfg
                .ipmi_sensors
                .iter()
                .filter_map(|s| s.warn_temp)
                .reduce(f64::min)
                .unwrap_or_else(|| {
                    error!(
                        controller = %ctrl_cfg.name,
                        "no warn_temp available for IPMI controller (must be in config)"
                    );
                    std::process::exit(1);
                }),
        };

        Some(SensorResult {
            sensors,
            pending,
            warn_temp,
        })
    } else {
        warn!(controller = %ctrl_cfg.name, "no hwmon_driver or ipmi_sensors configured, disabling");
        None
    }
}

fn log_status(outputs: &[ControllerOutput], min_duty_zone0: u8, min_duty_zone1: u8) {
    use std::collections::HashMap;

    struct ZoneInfo {
        details: Vec<String>,
        max_duty_output: Option<(String, f64, u8)>, // (controller_name, temp, duty)
    }

    let mut by_zone: HashMap<String, ZoneInfo> = HashMap::new();

    for o in outputs {
        let zone_name = match o.zone {
            config::ZoneId::Zone0 => "zone0",
            config::ZoneId::Zone1 => "zone1",
        };
        let info = by_zone
            .entry(zone_name.to_string())
            .or_insert_with(|| ZoneInfo {
                details: Vec::new(),
                max_duty_output: None,
            });

        // Track the controller with the highest duty for this zone
        let is_new_max = info
            .max_duty_output
            .as_ref()
            .is_none_or(|(_, _, d)| o.duty > *d);
        if is_new_max {
            info.max_duty_output = Some((o.controller_name.clone(), o.temp, o.duty));
        }

        let warn_marker = if o.warning { " WARN" } else { "" };

        // Format per-sensor detail
        let sensor_detail = if o.sensor_readings.is_empty() {
            String::new()
        } else if o.sensor_readings.len() > 4 {
            // Many sensors of same type — show count and max
            format!(
                " [×{}:max={:.0}]",
                o.sensor_readings.len(),
                o.sensor_readings
                    .iter()
                    .map(|r| r.temp)
                    .reduce(f64::max)
                    .unwrap_or(0.0)
            )
        } else {
            // Few distinct sensors — list each
            let parts: Vec<String> = o
                .sensor_readings
                .iter()
                .map(|r| format!("{}:{:.0}", r.label, r.temp))
                .collect();
            format!(" [{}]", parts.join(","))
        };

        info.details.push(format!(
            "{}={:.1}°C→{}%{}{}",
            o.controller_name, o.temp, o.duty, warn_marker, sensor_detail
        ));
    }

    for (zone, zone_info) in &by_zone {
        let floor = match zone.as_str() {
            "zone0" => min_duty_zone0,
            "zone1" => min_duty_zone1,
            _ => 0,
        };

        let (driver, effective_duty) = match &zone_info.max_duty_output {
            Some((name, temp, duty)) if *duty >= floor => (format!("{name}:{temp:.1}°C"), *duty),
            Some((_, _, duty)) => {
                // Floor is higher than any controller output
                (format!("floor:{floor}%"), floor.max(*duty))
            }
            None => ("none".to_string(), floor),
        };

        info!(
            duty = effective_duty,
            driver = driver.as_str(),
            zone = zone.as_str(),
            details = zone_info.details.join(", ").as_str(),
            "status"
        );
    }
}
