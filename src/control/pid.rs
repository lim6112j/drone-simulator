/// 미분항은 오차 미분(prev_err 사용), 적분항은 조건부 anti-windup:
/// 이번 스텝 출력이 이미 포화 방향이고 오차도 같은 부호면 적분을 멈춘다.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn p_only_gain() {
        let mut pid = Pid::new(2.0, 0.0, 0.0, -100.0, 100.0);
        let out = pid.step(1.0, 0.001);
        assert!((out - 2.0).abs() < 1e-6, "P-only output was {out}");
    }

    #[test]
    fn integral_accumulates_over_time() {
        let mut pid = Pid::new(0.0, 1.0, 0.0, -100.0, 100.0);
        let mut last = 0.0;
        for _ in 0..1000 {
            last = pid.step(1.0, 0.001);
        }
        assert!(last > 0.5, "integral should have accumulated, last={last}");
        assert!(pid.integral > 0.9 && pid.integral < 1.1, "integral={}", pid.integral);
    }

    #[test]
    fn anti_windup_limits_integral() {
        let mut pid = Pid::new(1.0, 1.0, 0.0, -1.0, 1.0);
        for _ in 0..10_000 {
            pid.step(10.0, 0.001);
        }
        assert!(pid.integral <= 1.0 + 1e-5, "integral winded up to {}", pid.integral);
    }

    #[test]
    fn reset_clears_state() {
        let mut pid = Pid::new(1.0, 1.0, 0.5, -10.0, 10.0);
        pid.step(1.0, 0.001);
        pid.reset();
        assert_eq!(pid.integral, 0.0, "integral must be cleared");
        assert_eq!(pid.prev_err, 0.0, "prev_err must be cleared");
    }
}
#[derive(Debug, Clone)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral: f32,
    pub prev_err: f32,
    pub i_min: f32,
    pub i_max: f32,
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32, i_min: f32, i_max: f32) -> Self {
        Self { kp, ki, kd, integral: 0.0, prev_err: 0.0, i_min, i_max }
    }

    pub fn step(&mut self, err: f32, dt: f32) -> f32 {
        // 비례항
        let p = self.kp * err;
        // 미분항 (오차 미분)
        let d = if self.kd != 0.0 {
            self.kd * (err - self.prev_err) / dt
        } else {
            0.0
        };
        // 적분항 후보
        let i_candidate = self.integral + self.ki * err * dt;
        let i_candidate = i_candidate.clamp(self.i_min, self.i_max);

        let mut out = p + i_candidate + d;

        // 조건부 anti-windup: 포화 방향으로 더 밀면 적분 중지
        let saturated_high = out > self.i_max && err > 0.0;
        let saturated_low = out < self.i_min && err < 0.0;
        if !(saturated_high || saturated_low) {
            self.integral = i_candidate;
            out = p + self.integral + d;
        }

        self.prev_err = err;
        out
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_err = 0.0;
    }
}
