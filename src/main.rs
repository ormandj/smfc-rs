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

use crate::controller::ThermalController;
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

    // Load config first (need log_level before initializing tracing)
    let cfg = match config::load(&cli.config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("fatal: failed to load config: {e}");
            std::process::exit(1);
        }
    };

    // Initialize tracing
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

    // Open IPMI device
    let ipmi_dev = if !cli.dry_run {
        match ipmi::IpmiDevice::open(&cfg.general.ipmi_device) {
            Ok(dev) => Some(dev),
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

    // Register SIGTERM handler
    let shutdown = Arc::new(AtomicBool::new(false));
    {
        let shutdown = Arc::clone(&shutdown);
        signal_hook::flag::register(signal_hook::consts::SIGTERM, shutdown)
            .expect("failed to register SIGTERM handler");
    }
    {
        let shutdown = Arc::clone(&shutdown);
        signal_hook::flag::register(signal_hook::consts::SIGINT, shutdown)
            .expect("failed to register SIGINT handler");
    }

    // Discover sensors and build controllers
    let mut controllers: Vec<ThermalController> = Vec::new();
    for ctrl_cfg in &cfg.controllers {
        if !ctrl_cfg.enabled {
            info!(controller = %ctrl_cfg.name, "disabled, skipping");
            continue;
        }

        let sensors = match hwmon::discover_sensors(&ctrl_cfg.hwmon_driver) {
            Ok(s) => s,
            Err(e) => {
                warn!(controller = %ctrl_cfg.name, error = %e, "sensor discovery failed, disabling");
                continue;
            }
        };

        if sensors.is_empty() {
            warn!(controller = %ctrl_cfg.name, driver = %ctrl_cfg.hwmon_driver, "no sensors found, disabling");
            continue;
        }

        // Determine effective warn_temp
        let warn_temp = match hwmon::effective_warn_temp(&sensors, ctrl_cfg.pid.warn_temp) {
            Some(t) => t,
            None => {
                error!(
                    controller = %ctrl_cfg.name,
                    "no warn_temp available (not in hwmon temp*_max and not in config) — cannot safely control this device"
                );
                std::process::exit(1);
            }
        };

        info!(
            controller = %ctrl_cfg.name,
            driver = %ctrl_cfg.hwmon_driver,
            sensors = sensors.len(),
            zone = ?ctrl_cfg.zone,
            setpoint = format!("{:.0}°C", ctrl_cfg.pid.setpoint),
            warn_temp = format!("{warn_temp:.0}°C"),
            "initialized"
        );

        controllers.push(ThermalController::new(ctrl_cfg, sensors, warn_temp));
    }

    if controllers.is_empty() {
        error!("no controllers enabled — nothing to do");
        std::process::exit(1);
    }

    // Zone arbitrator
    let mut arbitrator =
        ZoneArbitrator::new(cfg.general.min_duty_zone0, cfg.general.min_duty_zone1);

    info!(controllers = controllers.len(), "starting control loop");

    // Main control loop
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

        // Check for warnings — any controller at warn_temp bypasses normal arbitration
        let any_warning = outputs.iter().any(|o| o.warning);
        if any_warning {
            // Force all zones with warnings to 100%
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

        // Normal arbitration for non-warning state
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

            // Periodic status log (every 30 seconds)
            log_counter += 1;
            if log_counter >= 30 || !updates.is_empty() {
                log_counter = 0;
                log_status(&outputs);
            }
        }
    }

    // Shutdown: restore saved fan mode
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

fn log_status(outputs: &[ControllerOutput]) {
    use std::collections::HashMap;
    let mut by_zone: HashMap<String, Vec<String>> = HashMap::new();

    for o in outputs {
        let zone_name = match o.zone {
            config::ZoneId::Zone0 => "zone0",
            config::ZoneId::Zone1 => "zone1",
        };
        let entry = by_zone.entry(zone_name.to_string()).or_default();
        let warn_marker = if o.warning { " WARN" } else { "" };
        entry.push(format!(
            "{}={:.1}°C→{}%{}",
            o.controller_name, o.temp, o.duty, warn_marker
        ));
    }

    for (zone, parts) in &by_zone {
        let max_duty = outputs
            .iter()
            .filter(|o| {
                let zn = match o.zone {
                    config::ZoneId::Zone0 => "zone0",
                    config::ZoneId::Zone1 => "zone1",
                };
                zn == zone
            })
            .map(|o| o.duty)
            .max()
            .unwrap_or(0);

        info!(
            zone = zone.as_str(),
            duty = max_duty,
            details = parts.join(", ").as_str(),
            "status"
        );
    }
}
