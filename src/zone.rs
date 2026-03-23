use std::collections::HashMap;

use crate::config::ZoneId;

/// Output from a single thermal controller for one poll cycle.
#[derive(Debug, Clone)]
pub struct ControllerOutput {
    pub zone: ZoneId,
    pub duty: u8,
    pub temp: f64,
    pub controller_name: String,
    pub warning: bool,
}

/// Arbitrates fan duty across multiple controllers targeting the same zone.
///
/// Uses max-wins: the highest duty requested by any controller for a zone
/// becomes that zone's duty. Clamps to the zone's minimum floor.
#[derive(Debug)]
pub struct ZoneArbitrator {
    min_duty: HashMap<ZoneId, u8>,
    last_duty: HashMap<ZoneId, u8>,
}

/// A zone update to be sent via IPMI.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZoneUpdate {
    pub zone: ZoneId,
    pub duty: u8,
}

impl ZoneArbitrator {
    pub fn new(min_zone0: u8, min_zone1: u8) -> Self {
        let mut min_duty = HashMap::new();
        min_duty.insert(ZoneId::Zone0, min_zone0);
        min_duty.insert(ZoneId::Zone1, min_zone1);
        Self {
            min_duty,
            last_duty: HashMap::new(),
        }
    }

    /// Given a set of controller outputs, compute per-zone duties and return
    /// only the zones whose duty has changed since the last call.
    pub fn arbitrate(&mut self, outputs: &[ControllerOutput]) -> Vec<ZoneUpdate> {
        // Group by zone, take max duty
        let mut zone_duty: HashMap<ZoneId, u8> = HashMap::new();
        for output in outputs {
            let entry = zone_duty.entry(output.zone.clone()).or_insert(0);
            *entry = (*entry).max(output.duty);
        }

        // Clamp to zone minimums (only for zones that have outputs)
        for (zone, duty) in &mut zone_duty {
            if let Some(min) = self.min_duty.get(zone) {
                *duty = (*duty).max(*min);
            }
        }

        // Only return updates where duty changed
        let mut updates = Vec::new();
        for (zone, duty) in &zone_duty {
            let last = self.last_duty.get(zone).copied().unwrap_or(0);
            if *duty != last {
                updates.push(ZoneUpdate {
                    zone: zone.clone(),
                    duty: *duty,
                });
                self.last_duty.insert(zone.clone(), *duty);
            }
        }

        updates
    }

    /// Force-set a zone to a specific duty (used for warn_temp override).
    /// Returns an update if the duty changed.
    pub fn force_zone(&mut self, zone: ZoneId, duty: u8) -> Option<ZoneUpdate> {
        let last = self.last_duty.get(&zone).copied().unwrap_or(0);
        if duty != last {
            self.last_duty.insert(zone.clone(), duty);
            Some(ZoneUpdate { zone, duty })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn output(zone: ZoneId, duty: u8, name: &str) -> ControllerOutput {
        ControllerOutput {
            zone,
            duty,
            temp: 50.0,
            controller_name: name.to_string(),
            warning: false,
        }
    }

    #[test]
    fn max_wins_same_zone() {
        let mut arb = ZoneArbitrator::new(25, 25);
        let outputs = vec![
            output(ZoneId::Zone0, 40, "cpu"),
            output(ZoneId::Zone0, 60, "gpu"),
            output(ZoneId::Zone0, 35, "nic"),
        ];
        let updates = arb.arbitrate(&outputs);
        assert_eq!(updates.len(), 1);
        assert_eq!(updates[0].zone, ZoneId::Zone0);
        assert_eq!(updates[0].duty, 60);
    }

    #[test]
    fn independent_zones() {
        let mut arb = ZoneArbitrator::new(25, 25);
        let outputs = vec![
            output(ZoneId::Zone0, 50, "cpu"),
            output(ZoneId::Zone1, 35, "hdd"),
        ];
        let updates = arb.arbitrate(&outputs);
        assert_eq!(updates.len(), 2);
    }

    #[test]
    fn clamps_to_minimum() {
        let mut arb = ZoneArbitrator::new(30, 25);
        let outputs = vec![output(ZoneId::Zone0, 20, "cpu")];
        let updates = arb.arbitrate(&outputs);
        assert_eq!(updates.len(), 1);
        assert_eq!(updates[0].duty, 30); // clamped to min_zone0
    }

    #[test]
    fn dedup_unchanged() {
        let mut arb = ZoneArbitrator::new(25, 25);
        let outputs = vec![output(ZoneId::Zone0, 50, "cpu")];

        let updates1 = arb.arbitrate(&outputs);
        assert_eq!(updates1.len(), 1);

        // Same outputs again — no change
        let updates2 = arb.arbitrate(&outputs);
        assert!(updates2.is_empty(), "expected no updates on unchanged duty");
    }

    #[test]
    fn force_zone_override() {
        let mut arb = ZoneArbitrator::new(25, 25);

        // First set via arbitrate
        let outputs = vec![output(ZoneId::Zone0, 50, "cpu")];
        arb.arbitrate(&outputs);

        // Force to 100%
        let update = arb.force_zone(ZoneId::Zone0, 100);
        assert!(update.is_some());
        assert_eq!(update.unwrap().duty, 100);

        // Force same value — no update
        let update = arb.force_zone(ZoneId::Zone0, 100);
        assert!(update.is_none());
    }
}
