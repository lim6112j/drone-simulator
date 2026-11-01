use nalgebra::{UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

pub const G: f32 = 9.81;

/// 기체 스펙. 기본값: 650 g 쿼드, 추력중비 약 2:1.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DroneParams {
    pub mass: f32,
    pub arm_length: f32,
    pub inertia: [f32; 3], // Ixx, Iyy, Izz
    pub tmax: f32,
    pub drag_lin: f32,
    pub drag_quad: f32,
    pub ang_damp: f32,
    pub yaw_tau_coef: f32,
}

impl Default for DroneParams {
    fn default() -> Self {
        Self {
            mass: 0.65,
            arm_length: 0.15,
            inertia: [0.02, 0.02, 0.04],
            tmax: 12.7,
            drag_lin: 0.1,
            drag_quad: 0.05,
            ang_damp: 0.1,
            yaw_tau_coef: 0.05,
        }
    }
}

/// 모터 인덱스: [FL, FR, RL, RR]. 축: x=전방, y=좌측, z=상방 (우수). 
/// 롤 토크(좌측 모터 상승 → 양), 피치 토크(전방 모터 상승 → 양), 요 토크(대각차).
#[derive(Debug, Clone)]
pub struct DroneState {
    pub pos: Vector3<f32>,
    pub vel: Vector3<f32>,
    pub quat: UnitQuaternion<f32>,
    pub omega_body: Vector3<f32>,
}

#[derive(Debug, Clone)]
pub struct Drone {
    pub state: DroneState,
}

impl Drone {
    pub fn new() -> Self {
        Self {
            state: DroneState {
                pos: Vector3::zeros(),
                vel: Vector3::zeros(),
                quat: UnitQuaternion::identity(),
                omega_body: Vector3::zeros(),
            },
        }
    }

    /// omegas: [FL, FR, RL, RR] 정규화 회전속도 0..1
    pub fn step(&mut self, omegas: [f32; 4], wind: Vector3<f32>, p: &DroneParams, dt: f32) {
        use crate::motor::thrust;
        let t_fl = thrust(omegas[0], p.tmax);
        let t_fr = thrust(omegas[1], p.tmax);
        let t_rl = thrust(omegas[2], p.tmax);
        let t_rr = thrust(omegas[3], p.tmax);
        let sum_t = t_fl + t_fr + t_rl + t_rr;

        let a = p.arm_length;
        // 스칼라 토크 크기: 롤(좌측 상승 +), 피치(전방 상승 +), 요(대각차)
        let tau_roll = a * (t_rl + t_fl) - a * (t_fr + t_rr);
        let tau_pitch = a * (t_fl + t_fr) - a * (t_rl + t_rr);
        let tau_yaw = p.yaw_tau_coef * (t_fl + t_rr - t_fr - t_rl);
        // 좌표계 변환: x=전방, y=좌측, z=상방에서 전방-상승 토크는 −y 회전이다.
        let tau = Vector3::new(tau_roll, -tau_pitch, tau_yaw);

        // 각속도 (기체 좌표계, 준-암시적)
        let i = Vector3::new(p.inertia[0], p.inertia[1], p.inertia[2]);
        let w_new = self.state.omega_body + component_div(tau, &i) * dt
            - self.state.omega_body * p.ang_damp * dt;

        // 자세 갱신 (body rates → 소각회전 합성)
        let dq = UnitQuaternion::from_scaled_axis(w_new * dt);
        self.state.quat = self.state.quat * dq;

        // 힘: 기체 z축 추력 + 중력 + 항력(상대풍)
        let thrust_world = self.state.quat * Vector3::new(0.0, 0.0, sum_t);
        let gravity = Vector3::new(0.0, 0.0, -p.mass * G);
        let v_rel = self.state.vel - wind;
        let speed = v_rel.norm();
        let drag = -v_rel * p.drag_lin - v_rel * (speed * p.drag_quad);
        let force = thrust_world + gravity + drag;

        // 준-암시적 오일러
        self.state.vel += force * (dt / p.mass);
        self.state.pos += self.state.vel * dt;
        self.state.omega_body = w_new;
    }
}

fn component_div(v: Vector3<f32>, i: &Vector3<f32>) -> Vector3<f32> {
    Vector3::new(v.x / i.x, v.y / i.y, v.z / i.z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn free_fall_under_gravity() {
        let mut d = Drone::new();
        // 항력을 0으로: 중력 자체만 검증
        let p = DroneParams { drag_lin: 0.0, drag_quad: 0.0, ..DroneParams::default() };
        for _ in 0..1000 {
            d.step([0.0; 4], Vector3::zeros(), &p, 0.001);
        }
        assert!((d.state.vel.z - (-9.81)).abs() < 0.3, "vel={}", d.state.vel);
    }

    #[test]
    fn hover_thrust_holds_altitude() {
        let mut d = Drone::new();
        let p = DroneParams::default();
        let w = (p.mass * 9.81 / (4.0 * p.tmax)).sqrt();
        for _ in 0..1000 {
            d.step([w; 4], Vector3::zeros(), &p, 0.001);
        }
        assert!(d.state.vel.z.abs() < 0.05, "vel.z={}", d.state.vel.z);
        assert!(d.state.pos.z.abs() < 0.05, "pos.z={}", d.state.pos.z);
    }

    #[test]
    fn differential_thrust_produces_roll() {
        let mut d = Drone::new();
        let p = DroneParams::default();
        // 0.1초만: 회전각이 π를 넘지 않는 범위에서 각속도 부호 확인
        for _ in 0..100 {
            d.step([1.0, 0.0, 1.0, 0.0], Vector3::zeros(), &p, 0.001); // FL, RL 최대
        }
        assert!(d.state.omega_body.x > 0.5,
            "roll rate should be positive (left side up), got {}", d.state.omega_body.x);
        let euler = d.state.quat.euler_angles();
        assert!(euler.0 > 0.0, "roll angle must also be positive, got {}", euler.0);
    }

    #[test]
    fn wind_pushes_drone_downwind() {
        let mut d = Drone::new();
        let p = DroneParams::default();
        let wind = Vector3::new(5.0, 0.0, 0.0);
        for _ in 0..100 {
            d.step([0.0; 4], wind, &p, 0.001);
        }
        assert!(d.state.vel.x > 0.1, "must accelerate downwind, vel={}", d.state.vel);
    }
}
