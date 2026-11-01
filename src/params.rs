use serde::{Deserialize, Serialize};

/// 전역 시뮬레이션 파라미터. Task 8에서 실제 구조로 채워짐.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Params {
    pub placeholder: f32,
}
