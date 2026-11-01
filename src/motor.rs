use std::collections::VecDeque;

/// 모터: 순수 지연(dead time) + 1차 시간 지연. 명령 u∈[0,1] → 정규화 회전속도 ω∈[0,1].
#[derive(Debug, Clone)]
pub struct Motor {
    delay_line: VecDeque<f32>,
    pub delay_steps: usize,
    state: f32,
    tau_s: f32,
    dt: f32,
}

impl Motor {
    pub fn new(delay_s: f32, tau_s: f32, dt: f32) -> Self {
        let mut m = Self {
            delay_line: VecDeque::new(),
            delay_steps: 0,
            state: 0.0,
            tau_s,
            dt,
        };
        m.set_delay(delay_s);
        m
    }

    /// 지연 변경 (실행 중 슬라이더 반영용). 상태는 보존되고 버퍼만 리사이즈된다.
    pub fn set_delay(&mut self, delay_s: f32) {
        let steps = ((delay_s / self.dt).round() as i64).max(0) as usize;
        self.delay_steps = steps;
        self.delay_line.resize(steps, 0.0);
    }

    /// 시상수 변경 (실행 중 슬라이더 반영용).
    pub fn set_tau(&mut self, tau_s: f32) {
        self.tau_s = tau_s;
    }

    pub fn step(&mut self, command: f32) -> f32 {
        let delayed = if self.delay_steps == 0 {
            command
        } else {
            self.delay_line.push_front(command);
            if self.delay_line.len() > self.delay_steps {
                self.delay_line.pop_back().unwrap()
            } else {
                0.0 // 히스토리가 아직 차지 않았으면 정지 상태에서 시작
            }
        };
        let alpha = if self.tau_s > 0.0 { self.dt / self.tau_s } else { 1.0 };
        let alpha = alpha.min(1.0);
        self.state += (delayed - self.state) * alpha;
        self.state.clamp(0.0, 1.0)
    }

    pub fn omega(&self) -> f32 {
        self.state
    }

    pub fn tau(&self) -> f32 {
        self.tau_s
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
        self.delay_line.clear();
        self.delay_line.resize(self.delay_steps, 0.0);
    }
}

/// 추력 = k·ω² (k = tmax, ω 정규화), 클램프 [0, tmax].
pub fn thrust(omega: f32, tmax: f32) -> f32 {
    let w = omega.clamp(0.0, 1.0);
    (tmax * w * w).clamp(0.0, tmax)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    #[test]
    fn pure_delay_holds_output_then_responds() {
        let mut m = Motor::new(0.05, 0.0, DT); // 50스텝 지연, 시상수 0
        for i in 1..=60 {
            let w = m.step(1.0);
            if i <= 50 {
                assert_eq!(w, 0.0, "step {i}: must still be 0 during dead time");
            } else {
                assert!((w - 1.0).abs() < 1e-6, "step {i}: tau=0 so must jump to 1, got {w}");
            }
        }
    }

    #[test]
    fn first_order_lag_reaches_63pct_at_tau() {
        let mut m = Motor::new(0.0, 0.1, DT);
        for _ in 0..100 {
            m.step(1.0);
        }
        // 0.1 s = 시상수 1개 → 1 - e^-1 ≈ 0.632
        assert!((m.omega() - 0.632).abs() < 0.02, "omega={}", m.omega());
    }

    #[test]
    fn zero_delay_equals_direct_first_order() {
        let mut a = Motor::new(0.0, 0.05, DT);
        let mut b = Motor::new(0.0001, 0.05, DT); // 0스텝으로 양자화됨
        for i in 0..300 {
            let cmd = if i < 100 { 1.0 } else { 0.3 };
            let wa = a.step(cmd);
            let wb = b.step(cmd);
            assert!((wa - wb).abs() < 1e-6, "step {i}: {wa} vs {wb}");
        }
    }

    #[test]
    fn set_delay_preserves_state() {
        let mut m = Motor::new(0.02, 0.05, DT);
        for _ in 0..200 {
            m.step(1.0);
        }
        let before = m.omega();
        m.set_delay(0.05);
        assert_eq!(m.delay_steps, 50, "delay_steps must be re-quantized");
        assert!((m.omega() - before).abs() < 1e-6, "motor state must survive resize");
    }

    #[test]
    fn thrust_is_k_omega_squared_clamped() {
        assert!((thrust(0.5, 12.0) - 3.0).abs() < 1e-5);
        assert_eq!(thrust(-1.0, 12.0), 0.0, "negative omega clamps to zero thrust");
        assert!((thrust(2.0, 12.0) - 12.0).abs() < 1e-5, "omega above 1 saturates at tmax");
    }

    #[test]
    fn reset_clears_motor() {
        let mut m = Motor::new(0.01, 0.05, DT);
        for _ in 0..100 {
            m.step(1.0);
        }
        m.reset();
        assert_eq!(m.omega(), 0.0);
        let w = m.step(1.0);
        assert_eq!(w, 0.0, "after reset the delay line must be empty too");
    }
}
