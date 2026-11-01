use crate::control::pid::Pid;
use crate::drone::{DroneState, G};
use crate::params::Params;
use nalgebra::Vector3;

/// 제어 목표 (세계 좌표계 위치 + 요 각).
#[derive(Debug, Clone, Copy)]
pub struct Target {
    pub pos: Vector3<f32>,
    pub yaw: f32,
}

/// 컨트롤러 교체 인터페이스 (추후 RL/LQR 확장 여지).
pub trait Controller {
    fn step(&mut self, state: &DroneState, target: &Target, params: &Params, dt: f32) -> [f32; 4];
    fn reset(&mut self);
}

/// 캐스케이드 PID: 위치 P → 속도 PID(3축) → 수직 추력/목표 기울기 →
/// 자세 P → 목표 각속도 → 각속도 PID(3축, 토크) → 모터 믹싱.
///
/// 축 규약 (UI에 표시됨): x=전방, y=좌측, z=상방. 롤=+x 회전(좌측 상승),
/// 피치=+y 회전(기수 하강), 요=+z 회전(기수 좌회전).
pub struct CascadeController {
    vel: [Pid; 3],
    rate: [Pid; 3],
    yaw_rate: Pid,
}

impl Default for CascadeController {
    fn default() -> Self {
        Self::new()
    }
}

impl CascadeController {
    pub fn new() -> Self {
        Self {
            vel: std::array::from_fn(|_| Pid::new(0.0, 0.0, 0.0, -5.0, 5.0)),
            rate: std::array::from_fn(|_| Pid::new(0.0, 0.0, 0.0, -1.5, 1.5)),
            yaw_rate: Pid::new(0.0, 0.0, 0.0, -1.5, 1.5),
        }
    }

    /// 매 스텝 게인을 Params에서 동기화 (라이브 슬라이더 튜닝 반영).
    fn sync_gains(&mut self, g: &crate::params::Gains) {
        for v in &mut self.vel {
            v.kp = g.vel_kp;
            v.ki = g.vel_ki;
            v.kd = g.vel_kd;
        }
        for r in &mut self.rate {
            r.kp = g.rate_kp;
            r.ki = g.rate_ki;
            r.kd = g.rate_kd;
        }
        self.yaw_rate.kp = g.yaw_rate_kp;
        self.yaw_rate.ki = g.yaw_rate_ki;
    }
}

/// 모터 믹싱: 콜렉티브 추력 + 3축 토크(기체 좌표계) → 4모터 정규화 회전속도 [FL, FR, RL, RR].
/// τ_x=롤(좌측+), τ_y=피치(기수 하강+), τ_z=요(기수 좌회전+).
pub fn mix(thrust: f32, tau: Vector3<f32>, p: &crate::drone::DroneParams) -> [f32; 4] {
    let a = p.arm_length;
    let t4 = thrust / 4.0;
    let dp = tau.y / (4.0 * a); // 후방 +, 전방 − (기수 하강 = 후방 모터 강함)
    let dr = tau.x / (4.0 * a); // 좌측 +
    let dy = if p.yaw_tau_coef > 1e-6 { tau.z / (4.0 * p.yaw_tau_coef) } else { 0.0 };
    let ti = [
        t4 - dp + dr + dy, // FL
        t4 - dp - dr - dy, // FR
        t4 + dp + dr - dy, // RL
        t4 + dp - dr + dy, // RR
    ];
    let mut out = [0.0f32; 4];
    for (o, t) in out.iter_mut().zip(ti) {
        *o = (t.max(0.0) / p.tmax).sqrt().clamp(0.0, 1.0);
    }
    out
}

fn wrap_pi(a: f32) -> f32 {
    let mut x = a;
    while x > std::f32::consts::PI {
        x -= 2.0 * std::f32::consts::PI;
    }
    while x < -std::f32::consts::PI {
        x += 2.0 * std::f32::consts::PI;
    }
    x
}

impl Controller for CascadeController {
    fn step(&mut self, state: &DroneState, target: &Target, params: &Params, dt: f32) -> [f32; 4] {
        let g = params.gains.clone();
        self.sync_gains(&g);
        let (roll, pitch, yaw) = state.quat.euler_angles();

        // 1) 위치 P → 속도 명령
        let e_pos = target.pos - state.pos;
        let v_cmd = Vector3::new(
            (g.pos_kp * e_pos.x).clamp(-g.vmax, g.vmax),
            (g.pos_kp * e_pos.y).clamp(-g.vmax, g.vmax),
            (g.pos_kp * e_pos.z).clamp(-g.vmax, g.vmax),
        );

        // 2) 속도 PID → 가속 명령
        let e_vel = v_cmd - state.vel;
        let ax = self.vel[0].step(e_vel.x, dt).clamp(-g.amax, g.amax);
        let ay = self.vel[1].step(e_vel.y, dt).clamp(-g.amax, g.amax);
        let az = self.vel[2].step(e_vel.z, dt).clamp(-g.amax, g.amax);

        // 3) 수직: 콜렉티브 추력 (기울기 보상 포함)
        let body_z = state.quat * Vector3::new(0.0, 0.0, 1.0);
        let cos_tilt = body_z.z.max(0.5);
        let thrust =
            ((params.drone.mass * (G + az)) / cos_tilt).clamp(0.0, params.drone.tmax * 0.98);

        // 4) 수평: 가속 명령 → 목표 기울기 (기수 하강 양의 피치 = +x 가속)
        let tilt_max = g.tilt_max_deg.to_radians();
        let pitch_des = ax.atan2(G).clamp(-tilt_max, tilt_max);
        let roll_des = (-ay).atan2(G).clamp(-tilt_max, tilt_max);

        // 5) 자세 P → 목표 각속도
        let w_des_x = (g.att_kp * wrap_pi(roll_des - roll)).clamp(-g.rate_max, g.rate_max);
        let w_des_y = (g.att_kp * wrap_pi(pitch_des - pitch)).clamp(-g.rate_max, g.rate_max);
        let w_des_z = (g.yaw_att_kp * wrap_pi(target.yaw - yaw)).clamp(-g.rate_max, g.rate_max);

        // 6) 각속도 PID → 토크
        let tau = Vector3::new(
            self.rate[0].step(w_des_x - state.omega_body.x, dt),
            self.rate[1].step(w_des_y - state.omega_body.y, dt),
            self.yaw_rate.step(w_des_z - state.omega_body.z, dt),
        );

        // 7) 믹싱
        mix(thrust, tau, &params.drone)
    }

    fn reset(&mut self) {
        for v in &mut self.vel {
            v.reset();
        }
        for r in &mut self.rate {
            r.reset();
        }
        self.yaw_rate.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drone::Drone;
    use crate::motor::Motor;
    use crate::wind::WindGen;

    const DT: f32 = 0.001;

    struct Rig {
        drone: Drone,
        motors: [Motor; 4],
        ctrl: CascadeController,
        wind: WindGen,
        params: Params,
    }

    fn rig() -> Rig {
        let mut params = Params::default();
        params.motor.delay_s = 0.02;
        params.motor.tau_s = 0.05;
        Rig {
            drone: Drone::new(),
            motors: std::array::from_fn(|_| {
                Motor::new(params.motor.delay_s, params.motor.tau_s, DT)
            }),
            ctrl: CascadeController::new(),
            wind: WindGen::new(),
            params,
        }
    }

    fn wind_apply(rig: &mut Rig) {
        let w = &rig.params.wind;
        rig.wind.set(
            Vector3::new(w.base[0], w.base[1], w.base[2]),
            w.gust,
            w.amplitude,
            w.period_s,
        );
    }

    /// seconds 동안 굴리고 (최종 위치, 스텝 이후 최대 z) 반환.
    fn simulate(
        rig: &mut Rig,
        seconds: f32,
        target_at: impl Fn(f32) -> Target,
    ) -> (Vector3<f32>, f32) {
        wind_apply(rig);
        let steps = (seconds / DT) as usize;
        let mut max_z_after_step = f32::MIN;
        for i in 0..steps {
            let t = i as f32 * DT;
            let target = target_at(t);
            let state = rig.drone.state.clone();
            let cmd = rig.ctrl.step(&state, &target, &rig.params, DT);
            let mut omegas = [0.0f32; 4];
            for (j, m) in rig.motors.iter_mut().enumerate() {
                m.step(cmd[j]);
                omegas[j] = m.omega();
            }
            let wind = rig.wind.step(DT);
            let dp = rig.params.drone.clone();
            rig.drone.step(omegas, wind, &dp, DT);
            if t >= rig.params.sim.step_time_s {
                max_z_after_step = max_z_after_step.max(rig.drone.state.pos.z);
            }
        }
        (rig.drone.state.pos, max_z_after_step)
    }

    #[test]
    fn hover_rejects_steady_wind_via_integral() {
        let mut r = rig();
        r.params.wind.base = [3.0, 0.0, 0.0];
        let (pos, _) = simulate(&mut r, 10.0, |_| Target { pos: Vector3::zeros(), yaw: 0.0 });
        let horiz_err = (pos.x * pos.x + pos.y * pos.y).sqrt();
        assert!(horiz_err < 0.05, "horizontal error {horiz_err} m after 10 s");
        assert!(pos.z.abs() < 0.05, "z error {} m", pos.z);
    }

    #[test]
    fn vertical_step_settles_without_big_overshoot() {
        let mut r = rig();
        let h = r.params.sim.step_height_m;
        let ts = r.params.sim.step_time_s;
        let (pos, max_z) = simulate(&mut r, 8.0, |t| {
            let z = if t >= ts { h } else { 0.0 };
            Target { pos: Vector3::new(0.0, 0.0, z), yaw: 0.0 }
        });
        assert!((pos.z - h).abs() < 0.1, "final z error {} m", (pos.z - h).abs());
        assert!(max_z - h < 0.3, "overshoot {} m", max_z - h);
    }

    #[test]
    fn horizontal_step_settles() {
        let mut r = rig();
        let d = r.params.sim.step_dist_m;
        let ts = r.params.sim.step_time_s;
        let (pos, _) = simulate(&mut r, 10.0, |t| {
            let x = if t >= ts { d } else { 0.0 };
            Target { pos: Vector3::new(x, 0.0, 0.0), yaw: 0.0 }
        });
        assert!((pos.x - d).abs() < 0.1, "final x error {} m", (pos.x - d).abs());
        assert!(pos.y.abs() < 0.1 && pos.z.abs() < 0.1, "cross-track drift {pos}");
    }

    #[test]
    fn zero_gains_do_not_track_step() {
        let mut r = rig();
        r.params.gains.pos_kp = 0.0;
        r.params.gains.vel_kp = 0.0;
        r.params.gains.vel_ki = 0.0;
        r.params.gains.vel_kd = 0.0;
        r.params.gains.att_kp = 0.0;
        r.params.gains.rate_kp = 0.0;
        r.params.gains.rate_ki = 0.0;
        r.params.gains.yaw_att_kp = 0.0;
        r.params.gains.yaw_rate_kp = 0.0;
        r.params.gains.yaw_rate_ki = 0.0;
        let d = r.params.sim.step_dist_m;
        let ts = r.params.sim.step_time_s;
        let (pos, _) = simulate(&mut r, 6.0, |t| {
            let x = if t >= ts { d } else { 0.0 };
            Target { pos: Vector3::new(x, 0.0, 0.0), yaw: 0.0 }
        });
        assert!((pos.x - d).abs() > 1.0, "zero gains must not track the step, pos={pos}");
    }
}
