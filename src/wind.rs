use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// 돌풍 종류. SmoothRandom은 1차 마르코프(저역통과 랜덤)로 흔들리는 바람을 흉내 낸다.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GustKind {
    None,
    Step,
    Sine,
    SmoothRandom,
}

/// 바람 생성기: 기본풍(벡터) + 돌풍(스칼라 진폭, 기본풍 방향) + 펄스(UI 버튼).
#[derive(Debug, Clone)]
pub struct WindGen {
    base: Vector3<f32>,
    gust_kind: GustKind,
    gust_amplitude: f32,
    gust_period_s: f32,
    gust_dir: Vector3<f32>,
    t: f32,
    state: f32, // SmoothRandom 내부 상태
    pulse_end: f32, // 절대 시각: t < pulse_end 동안 펄스 활성
    pulse_amp: f32,
    rng: u64, // LCG 시드 (결정론적)
}

impl Default for WindGen {
    fn default() -> Self {
        Self::new()
    }
}

impl WindGen {
    pub fn new() -> Self {
        Self {
            base: Vector3::zeros(),
            gust_kind: GustKind::None,
            gust_amplitude: 0.0,
            gust_period_s: 2.0,
            gust_dir: Vector3::new(1.0, 0.0, 0.0),
            t: 0.0,
            state: 0.0,
            pulse_end: 0.0,
            pulse_amp: 0.0,
            rng: 0x2545F4914F6CDD1D,
        }
    }

    pub fn set(&mut self, base: Vector3<f32>, kind: GustKind, amplitude: f32, period_s: f32) {
        self.base = base;
        self.gust_kind = kind;
        self.gust_amplitude = amplitude;
        self.gust_period_s = period_s.max(1e-3);
        // 돌풍 방향: 기본풍 방향, 없으면 +x
        self.gust_dir = if base.norm() > 1e-6 {
            base.normalize()
        } else {
            Vector3::new(1.0, 0.0, 0.0)
        };
    }

    fn rand_unit(&mut self) -> f32 {
        // LCG: [-1, 1] 결정론적 난수
        self.rng = self.rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.rng >> 33) as f32 / (u32::MAX as f32)) * 2.0 - 1.0
    }

    fn gust_scalar(&mut self, dt: f32) -> f32 {
        let a = self.gust_amplitude;
        match self.gust_kind {
            GustKind::None => 0.0,
            GustKind::Step => {
                if self.t % self.gust_period_s < self.gust_period_s * 0.5 {
                    a
                } else {
                    0.0
                }
            }
            GustKind::Sine => a * (2.0 * std::f32::consts::PI * self.t / self.gust_period_s).sin(),
            GustKind::SmoothRandom => {
                // 1차 마르코프: 목표값을 향해 완만히 이동, 스텝당 점프 제한
                let target = a * self.rand_unit();
                let alpha = (dt / (self.gust_period_s * 0.25)).min(1.0);
                self.state += (target - self.state) * alpha;
                self.state.clamp(-a, a)
            }
        }
    }

    pub fn step(&mut self, dt: f32) -> Vector3<f32> {
        self.t += dt;
        let g = self.gust_scalar(dt);
        let mut wind = self.base + self.gust_dir * g;
        if self.t < self.pulse_end {
            wind += self.gust_dir * self.pulse_amp;
        }
        wind
    }

    /// UI 바람 펄스 버튼: duration_s 동안 amplitude만큼 임시 돌풍.
    pub fn pulse(&mut self, amplitude: f32, duration_s: f32) {
        self.pulse_amp = amplitude;
        self.pulse_end = self.t + duration_s;
    }

    pub fn reset(&mut self) {
        self.t = 0.0;
        self.state = 0.0;
        self.pulse_end = 0.0;
        self.pulse_amp = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn base_only_when_gust_is_none() {
        let mut w = WindGen::new();
        w.set(Vector3::new(3.0, 0.0, 0.0), GustKind::None, 5.0, 2.0);
        for _ in 0..100 {
            assert_eq!(w.step(0.001), Vector3::new(3.0, 0.0, 0.0));
        }
    }

    #[test]
    fn step_gust_lasts_half_period_then_returns() {
        let mut w = WindGen::new();
        w.set(Vector3::zeros(), GustKind::Step, 4.0, 2.0); // 1초간 +4 m/s
        let mid = w.step(0.5); // t=0.5 < period/2=1.0
        assert!((mid.x - 4.0).abs() < 1e-5, "mid={mid}");
        let after = w.step(0.6); // t=1.1 ≥ period/2
        assert!(after.x.abs() < 1e-5, "after={after}");
    }

    #[test]
    fn sine_gust_peaks_at_quarter_period() {
        let mut w = WindGen::new();
        w.set(Vector3::new(1.0, 0.0, 0.0), GustKind::Sine, 2.0, 4.0);
        let _ = w.step(1.0); // t=1.0 = period/4 → sin(π/2)=1 → base+2
        let peak = w.step(1e-9);
        assert!((peak.x - 3.0).abs() < 0.01, "peak={peak}");
        // t=2.0 = period/2 → sin(π)=0 → base만
        let mut w2 = WindGen::new();
        w2.set(Vector3::new(1.0, 0.0, 0.0), GustKind::Sine, 2.0, 4.0);
        let _ = w2.step(2.0);
        let at_half = w2.step(1e-9);
        assert!((at_half.x - 1.0).abs() < 0.01, "at_half={at_half}");
    }

    #[test]
    fn pulse_adds_temporary_gust() {
        let mut w = WindGen::new();
        w.set(Vector3::zeros(), GustKind::None, 0.0, 1.0);
        w.pulse(6.0, 1.0);
        let during = w.step(0.5);
        assert!((during.x - 6.0).abs() < 1e-5, "during={during}");
        let after = w.step(0.6); // 총 1.1 s > 1.0 s
        assert!(after.x.abs() < 1e-5, "after={after}");
    }

    #[test]
    fn smooth_random_never_jumps_beyond_amplitude() {
        let mut w = WindGen::new();
        w.set(Vector3::zeros(), GustKind::SmoothRandom, 5.0, 1.0);
        let mut prev = w.step(0.001);
        for _ in 0..2000 {
            let cur = w.step(0.001);
            assert!((cur.x - prev.x).abs() <= 5.0, "jump too large: {} -> {}", prev.x, cur.x);
            assert!(cur.x.abs() <= 5.0 + 1e-4, "beyond amplitude: {cur}");
            prev = cur;
        }
    }
}
