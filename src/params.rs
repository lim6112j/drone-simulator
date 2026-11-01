use crate::drone::DroneParams;
use crate::wind::GustKind;
use serde::{Deserialize, Serialize};

/// 모터 지연 파라미터.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorParams {
    pub delay_s: f32,
    pub tau_s: f32,
}

impl Default for MotorParams {
    fn default() -> Self {
        Self { delay_s: 0.02, tau_s: 0.05 }
    }
}

/// 캐스케이드 PID 게인. 출력 단위: 위치[m/s], 속도[m/s²], 자세[rad/s], 각속도[N·m].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Gains {
    pub pos_kp: f32,
    pub vel_kp: f32,
    pub vel_ki: f32,
    pub vel_kd: f32,
    pub att_kp: f32,
    pub rate_kp: f32,
    pub rate_ki: f32,
    pub rate_kd: f32,
    pub yaw_att_kp: f32,
    pub yaw_rate_kp: f32,
    pub yaw_rate_ki: f32,
    pub vmax: f32,
    pub amax: f32,
    pub rate_max: f32,
    pub tilt_max_deg: f32,
}

impl Default for Gains {
    fn default() -> Self {
        Self {
            pos_kp: 1.0,
            vel_kp: 2.0,
            vel_ki: 1.0,
            vel_kd: 0.0,
            att_kp: 3.0,
            rate_kp: 0.5,
            rate_ki: 1.5,
            rate_kd: 0.01,
            yaw_att_kp: 3.0,
            yaw_rate_kp: 0.4,
            yaw_rate_ki: 0.5,
            vmax: 2.5,
            amax: 5.0,
            rate_max: 8.0,
            tilt_max_deg: 30.0,
        }
    }
}

/// 바람 파라미터 (base는 m/s 벡터).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WindParams {
    pub base: [f32; 3],
    pub gust: GustKind,
    pub amplitude: f32,
    pub period_s: f32,
}

impl Default for WindParams {
    fn default() -> Self {
        Self { base: [0.0; 3], gust: GustKind::None, amplitude: 0.0, period_s: 2.0 }
    }
}

/// 미션 스텝 파라미터.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimParams {
    pub step_height_m: f32,
    pub step_dist_m: f32,
    pub step_time_s: f32,
}

impl Default for SimParams {
    fn default() -> Self {
        Self { step_height_m: 2.0, step_dist_m: 3.0, step_time_s: 2.0 }
    }
}

/// 전역 시뮬레이션 파라미터.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Params {
    pub drone: DroneParams,
    pub motor: MotorParams,
    pub gains: Gains,
    pub wind: WindParams,
    pub sim: SimParams,
}

/// 스모크 테스트 (Task 1)
#[cfg(test)]
mod smoke {
    use super::Params;

    #[test]
    fn params_default_constructs() {
        let p = Params::default();
        assert!(p.drone.mass > 0.0);
        assert!(p.motor.delay_s >= 0.0);
        assert!(p.gains.pos_kp > 0.0);
    }
}
