//! 고정 스텝 시뮬레이션 코어: 물리 + 모터 + 바람 + 컨트롤러를 묶어 실행하고 이력을 기록한다.
use crate::control::cascade::{CascadeController, Controller, Target};
use crate::drone::Drone;
use crate::motor::Motor;
use crate::params::Params;
use crate::wind::WindGen;
use nalgebra::Vector3;

pub const DT: f32 = 0.001;
const MAX_STEPS_PER_FRAME: u32 = 200;
const DIVERGE_POS_M: f32 = 1000.0;

/// 제어 과제.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mission {
    Hover,
    VerticalStep,
    HorizontalStep,
}

/// 프론트엔드용 기록 버퍼 (UI에서 다운샘플).
#[derive(Debug, Clone, Default)]
pub struct History {
    pub t: Vec<f32>,
    pub pos_err: Vec<f32>,
    pub pos: Vec<[f32; 3]>,
    pub roll_pitch_yaw: Vec<[f32; 3]>,
    pub motors: Vec<[f32; 4]>,
    pub wind: Vec<[f32; 3]>,
}

pub struct Sim {
    pub params: Params,
    drone: Drone,
    motors: [Motor; 4],
    wind: WindGen,
    controller: Box<dyn Controller>,
    pub history: History,
    pub t: f32,
    pub diverged: bool,
    pub mission: Mission,
    hover_origin: Vector3<f32>,
    carry: f32,
}

impl Sim {
    pub fn new(params: Params) -> Self {
        let origin = Vector3::new(0.0, 0.0, 0.0);
        let motors = std::array::from_fn(|_| {
            Motor::new(params.motor.delay_s, params.motor.tau_s, DT)
        });
        let mut wind = WindGen::new();
        let w = &params.wind;
        wind.set(
            Vector3::new(w.base[0], w.base[1], w.base[2]),
            w.gust,
            w.amplitude,
            w.period_s,
        );
        Sim {
            drone: Drone::new(),
            motors,
            controller: Box::new(CascadeController::new()),
            history: History::default(),
            t: 0.0,
            diverged: false,
            mission: Mission::Hover,
            params,
            hover_origin: origin,
            wind,
            carry: 0.0,
        }
    }

    /// 실행 중 파라미터 적용 (리셋 없음). 지연/시상수는 모터에, 바람은 생성기에 반영.
    pub fn apply_params(&mut self, params: Params) {
        for m in &mut self.motors {
            m.set_delay(params.motor.delay_s);
            m.set_tau(params.motor.tau_s);
        }
        let w = &params.wind;
        self.wind.set(
            Vector3::new(w.base[0], w.base[1], w.base[2]),
            w.gust,
            w.amplitude,
            w.period_s,
        );
        self.params = params;
    }

    /// 현재 기체 상태 (UI용).
    pub fn state(&self) -> &crate::drone::DroneState {
        &self.drone.state
    }

    /// 현재 미션의 목표 위치.
    pub fn target_pos(&self) -> Vector3<f32> {
        let s = &self.params.sim;
        match self.mission {
            Mission::Hover => self.hover_origin,
            Mission::VerticalStep => {
                if self.t >= s.step_time_s {
                    self.hover_origin + Vector3::new(0.0, 0.0, s.step_height_m)
                } else {
                    self.hover_origin
                }
            }
            Mission::HorizontalStep => {
                if self.t >= s.step_time_s {
                    self.hover_origin + Vector3::new(s.step_dist_m, 0.0, 0.0)
                } else {
                    self.hover_origin
                }
            }
        }
    }

    pub fn set_mission(&mut self, m: Mission) {
        self.mission = m;
    }

    pub fn wind_pulse(&mut self) {
        self.wind.pulse(5.0, 1.5);
    }

    pub fn reset(&mut self) {
        self.drone = Drone::new();
        for m in &mut self.motors {
            m.reset();
        }
        self.wind.reset();
        self.controller.reset();
        self.history = History::default();
        self.t = 0.0;
        self.diverged = false;
        self.carry = 0.0;
    }

    /// 실제 프레임 시간 × 배속만큼 물리를 진행한다.
    pub fn step_frame(&mut self, real_dt: f32, time_scale: f32) {
        if self.diverged {
            return;
        }
        self.carry += real_dt * time_scale;
        let mut n = (self.carry / DT) as u32;
        self.carry -= n as f32 * DT;
        if n > MAX_STEPS_PER_FRAME {
            n = MAX_STEPS_PER_FRAME;
            self.carry = 0.0;
        }
        for _ in 0..n {
            if self.diverged {
                break;
            }
            self.step_once();
        }
    }

    fn step_once(&mut self) {
        let target = Target { pos: self.target_pos(), yaw: 0.0 };
        let state = self.drone.state.clone();
        let cmd = self.controller.step(&state, &target, &self.params, DT);

        let mut omegas = [0.0f32; 4];
        for (j, m) in self.motors.iter_mut().enumerate() {
            m.step(cmd[j]);
            omegas[j] = m.omega();
        }

        let wind = self.wind.step(DT);
        let dp = self.params.drone.clone();
        self.drone.step(omegas, wind, &dp, DT);
        self.t += DT;

        // 발산 감지 — 기록 전에 확인해 NaN 유입을 차단
        let pos = self.drone.state.pos;
        let omega = self.drone.state.omega_body;
        let bad_pos = !pos.iter().all(|c| c.is_finite()) || pos.norm() > DIVERGE_POS_M;
        let bad_att = !omega.iter().all(|c| c.is_finite()) || omega.norm() > 200.0;
        if bad_pos || bad_att {
            self.diverged = true;
            return;
        }

        let err = (self.target_pos() - pos).norm();
        let t_now = self.t;
        let (r, p, y) = self.drone.state.quat.euler_angles();
        let wind_arr = [wind.x, wind.y, wind.z];
        let h = &mut self.history;
        h.t.push(t_now);
        h.pos_err.push(err);
        h.pos.push([pos.x, pos.y, pos.z]);
        h.roll_pitch_yaw.push([r, p, y]);
        h.motors.push(omegas);
        h.wind.push(wind_arr);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set_gains(p: &mut Params, mult: f32) {
        let g = &mut p.gains;
        g.pos_kp *= mult;
        g.vel_kp *= mult;
        g.vel_ki *= mult;
        g.vel_kd *= mult;
        g.att_kp *= mult;
        g.rate_kp *= mult;
        g.rate_ki *= mult;
        g.yaw_att_kp *= mult;
        g.yaw_rate_kp *= mult;
        g.yaw_rate_ki *= mult;
    }

    #[test]
    fn hover_holds_for_ten_seconds() {
        let mut sim = Sim::new(Params::default());
        for _ in 0..600 {
            sim.step_frame(1.0 / 60.0, 1.0);
        }
        assert!((sim.t - 10.0).abs() < 0.01, "t={}", sim.t);
        let pos = sim.drone.state.pos;
        assert!(pos.norm() < 0.3, "hover drifted to {pos}");
        assert!(!sim.diverged);
    }

    #[test]
    fn singular_params_flag_divergence_without_nan() {
        // 질량 0 → force*dt/mas → inf → 발산 플래그가 NaN 기록을 차단해야 함
        let mut sim = Sim::new(Params::default());
        sim.params.drone.mass = 0.0;
        sim.apply_params(sim.params.clone());
        for _ in 0..120 {
            sim.step_frame(1.0 / 60.0, 1.0);
            if sim.diverged {
                break;
            }
        }
        assert!(sim.diverged, "singular params must flag divergence");
        for e in &sim.history.pos_err {
            assert!(e.is_finite(), "NaN in history");
        }
    }

    #[test]
    fn insane_gains_stay_bounded_without_nan() {
        // 모터 포화 때문에 게인 100배도 유한한 리밋 사이클에 머문다 —
        // 앱이 죽지 않고 계속 실행 가능해야 함 (발산 플래그 없어도 정상).
        let mut sim = Sim::new(Params::default());
        set_gains(&mut sim.params, 100.0);
        sim.apply_params(sim.params.clone());
        for _ in 0..600 {
            sim.step_frame(1.0 / 60.0, 1.0);
            if sim.diverged {
                break;
            }
        }
        assert!(sim.drone.state.pos.norm().is_finite());
        for e in &sim.history.pos_err {
            assert!(e.is_finite(), "NaN in history");
        }
    }

    #[test]
    fn extreme_params_never_produce_nan() {
        let mut sim = Sim::new(Params::default());
        sim.params.motor.delay_s = 0.3;
        sim.params.motor.tau_s = 0.5;
        set_gains(&mut sim.params, 10.0);
        sim.params.drone.mass = 0.65;
        sim.apply_params(sim.params.clone());
        for _ in 0..300 {
            sim.step_frame(1.0 / 60.0, 1.0);
            if sim.diverged {
                break;
            }
        }
        for e in &sim.history.pos_err {
            assert!(e.is_finite(), "NaN in history");
        }
    }

    #[test]
    fn apply_params_updates_delay_without_reset() {
        let mut sim = Sim::new(Params::default());
        sim.step_frame(1.0 / 60.0, 1.0); // ~0.017 s 진행
        let pos_before = sim.drone.state.pos;
        let mut p2 = Params::default();
        p2.motor.delay_s = 0.05;
        p2.motor.tau_s = 0.08;
        sim.apply_params(p2);
        assert_eq!(sim.motors[0].delay_steps, 50, "delay must re-quantize");
        assert_eq!(sim.motors[0].tau(), 0.08, "tau must update live");
        assert_eq!(sim.drone.state.pos, pos_before, "state must survive apply_params");
    }

    #[test]
    fn horizontal_mission_changes_target_after_step_time() {
        let mut sim = Sim::new(Params::default());
        sim.set_mission(Mission::HorizontalStep);
        assert!((sim.target_pos().x - 0.0).abs() < 1e-6, "before step time");
        // 스텝 시각(2 s)까지 진행
        for _ in 0..2100 {
            sim.step_frame(DT, 1.0);
        }
        assert!(
            (sim.target_pos().x - sim.params.sim.step_dist_m).abs() < 1e-6,
            "after step time target must move"
        );
    }
}
