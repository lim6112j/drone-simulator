//! 원클릭 실험 프리셋.
use crate::params::Params;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    Tuned,
    DelayHeavy,
    SlowMotor,
    StrongWind,
    Unstable,
}

/// 프리셋의 한국어 라벨 (UI용).
pub fn preset_label(p: Preset) -> &'static str {
    match p {
        Preset::Tuned => "기준 튜닝",
        Preset::DelayHeavy => "지연 과다",
        Preset::SlowMotor => "느린 모터",
        Preset::StrongWind => "강풍",
        Preset::Unstable => "불안정",
    }
}

pub const ALL_PRESETS: [Preset; 5] = [
    Preset::Tuned,
    Preset::DelayHeavy,
    Preset::SlowMotor,
    Preset::StrongWind,
    Preset::Unstable,
];

pub fn preset_params(p: Preset) -> Params {
    let mut params = Params::default();
    match p {
        Preset::Tuned => {}
        Preset::DelayHeavy => {
            params.motor.delay_s = 0.15;
            params.motor.tau_s = 0.10;
        }
        Preset::SlowMotor => {
            params.motor.delay_s = 0.02;
            params.motor.tau_s = 0.30;
        }
        Preset::StrongWind => {
            params.wind.base = [5.0, 0.0, 0.0];
            params.wind.gust = crate::wind::GustKind::Step;
            params.wind.amplitude = 3.0;
            params.wind.period_s = 4.0;
        }
        Preset::Unstable => {
            // 교육 메시지: "지연이 커지면 기본 튜닝으로도 붕괴한다"
            params.motor.delay_s = 0.25;
            params.motor.tau_s = 0.10;
        }
    }
    params
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::cascade::Target;
    use crate::sim::{Sim, DT};
    use nalgebra::Vector3;

    #[test]
    fn toml_roundtrip_preserves_values() {
        let mut p = Params::default();
        p.motor.delay_s = 0.07;
        p.gains.vel_kp = 2.3;
        p.wind.base = [1.5, -2.0, 0.0];
        p.wind.gust = crate::wind::GustKind::Sine;
        let s = p.to_toml();
        let q = Params::from_toml(&s).expect("parse must succeed");
        assert_eq!(q.motor.delay_s, 0.07);
        assert_eq!(q.gains.vel_kp, 2.3);
        assert_eq!(q.wind.base, [1.5, -2.0, 0.0]);
        assert_eq!(q.wind.gust, crate::wind::GustKind::Sine);
        assert_eq!(q.drone.mass, p.drone.mass);
    }

    #[test]
    fn presets_are_distinct() {
        let sig = |p: Preset| {
            let q = preset_params(p);
            (q.motor.delay_s, q.motor.tau_s, q.gains.vel_kp, q.wind.base[0])
        };
        let vals: Vec<_> = ALL_PRESETS.iter().map(|&p| (p, sig(p))).collect();
        for i in 0..vals.len() {
            for j in (i + 1)..vals.len() {
                assert_ne!(vals[i].1, vals[j].1, "{:?} vs {:?} identical", vals[i].0, vals[j].0);
            }
        }
    }

    #[test]
    fn tuned_preset_passes_hover_wind_scenario() {
        let mut params = preset_params(Preset::Tuned);
        params.wind.base = [3.0, 0.0, 0.0];
        let mut sim = Sim::new(params);
        for _ in 0..600 {
            sim.step_frame(1.0 / 60.0, 1.0);
        }
        let pos = sim.state().pos;
        let err = (pos.x * pos.x + pos.y * pos.y).sqrt();
        assert!(err < 0.05, "tuned preset wind rejection failed: {err}");
    }

    #[test]
    fn unstable_preset_oscillates_visibly() {
        let mut sim = Sim::new(preset_params(Preset::Unstable));
        for _ in 0..600 {
            sim.step_frame(1.0 / 60.0, 1.0);
        }
        let max_err = sim
            .history
            .pos_err
            .iter()
            .cloned()
            .fold(0.0f32, f32::max);
        assert!(max_err > 1.0, "unstable preset must visibly oscillate, max_err={max_err}");
        assert!(sim.diverged || max_err > 1.0);
        let _ = Target { pos: Vector3::zeros(), yaw: 0.0 }; // import 유지
        let _ = DT;
    }
}
