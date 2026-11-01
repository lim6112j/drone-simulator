# 드론 제어 시뮬레이터 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 모터 지연이 드론 제어(호버/수직/수평 이동, 바람 상쇄)에 주는 영향을 슬라이더로 즉시 실험하는 Rust 네이티브 GUI 시뮬레이터.

**Architecture:** 순수 로직(물리·모터·바람·PID·캐스케이드 컨트롤러·시뮬레이터 코어)은 유닛 테스트 가능한 라이브러리 모듈로, egui UI는 얇게 유지. 물리는 고정 스텝 1 kHz, 모터 지연은 링버퍼 지연 라인 + 1차 시상수로 모델링. 컨트롤러는 trait 뒤에 위치(RL 확장 여지).

**Tech Stack:** Rust (stable), eframe/egui + egui_plot (eframe 0.33 계열 최신, egui_plot은 그 egui와 호환되는 최신 버전 — docs.rs에서 짝 확인), nalgebra (Vector3, UnitQuaternion), serde + toml.

**Spec:** `docs/superpowers/specs/2025-11-01-drone-control-sim-design.md`

## Global Constraints

- 단위는 SI (m, s, kg, N, rad)
- 물리 고정 스텝 `DT = 0.001` s (1 kHz); UI 프레임마다 `time_scale` 배수만큼 스텝 실행
- 모터 순수 지연은 `DT` 정수 배로 양자화, 링버퍼 구현
- 모든 PID에 anti-windup 필수 (조건부 적분 또는 back-calculation)
- 모터 추력 포화 `[0, Tmax]`, 추력 = `k * ω²` (ω 정규화 0..1, k = Tmax)
- 컨트롤러는 `Controller` trait 뒤에 위치 (RL 등 확장 여지 — v1에서는 PID만)
- 자세는 쿼터니온 (`nalgebra::UnitQuaternion<f32>`)
- UI 텍스트는 한국어
- 네트워크 사용 없음, 단일 실행파일 (`cargo run --release`로 실행)
- 바람은 상대풍으로 기체에 작용 (기체 속도와의 차로 저항 계산)

## Review Focus

1. **발산이 앱을 죽지 않게** — 지연 300 ms + 기본 게인 등으로 상태가 발산하면(`|pos| > 1000 m` 또는 NaN) 시뮬레이션이 멈추고 UI에 "발산" 표시. → Task 7의 발산 플래그 테스트.
2. **지연 0이 기준선과 동일** — `delay_s = 0`일 때 결과가 지연 라인 없는 1차 지연과 동일. → Task 3의 zero-delay 테스트.
3. **포화에서 적분 폭주 금지** — 명령이 포화로 클램프되는 동안 적분기가 계속 자라지 않음. → Task 2의 anti-windup 테스트.
4. **슬라이더 실시간 반영** — 실행 중 파라미터 변경(지연·게인·바람)이 리셋 없이 다음 스텝부터 적용. → Task 8의 live-apply 테스트 (게인) + Task 7의 지연 버퍼 리사이즈 테스트.
5. **극단 파라미터에서 NaN 금지** — 어떤 슬라이더 조합으로도 물리 스텝이 NaN을 만들지 않음 (내부 클램프). → Task 7의 극단 파라미터 테스트.

---

### Task 1: 프로젝트 골격

**Files:**
- Create: `Cargo.toml`, `src/main.rs`, `src/lib.rs`, `src/params.rs` (빈 `Params` 구조만)
- Test: `src/lib.rs` 내 스모크 테스트

**Interfaces:**
- Produces: 크레이트 `drone-training`, 모듈 트리 `params`, `motor`, `wind`, `drone`, `control::{pid, cascade}`, `sim`, `presets`, `viz`, `app` (이후 태스크가 채움)

- [ ] **Step 1: 초기화**

```bash
cd /Users/nixos/workspace/drone-training && git init && cargo init --name drone-training
```

- [ ] **Step 2: 의존성 추가** — `Cargo.toml`에 `eframe`(0.33 계열 최신), `egui_plot`(호환 최신), `nalgebra`(f32 features), `serde`(derive), `toml`. 버전 짝은 docs.rs에서 확인.

- [ ] **Step 3: 모듈 뼈대 작성** — `lib.rs`에 위 모듈 선언 + 스모크 테스트 1개 (예: `assert_eq!(2+2, 4)` 수준이 아니라 `params` 모듈이 존재하고 기본 구조체가 생성됨). `params.rs`에 `#[derive(Debug, Clone, Serialize, Deserialize)] pub struct Params` 빼곡한 채로 두지 말고 Task 8에서 채울 것이므로 지금은 `#[derive(...)] pub struct Params { pub placeholder: f32 }` 수준.

- [ ] **Step 4: 빌드·테스트 확인**

Run: `cargo test`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "chore: project skeleton with module tree"
```

---

### Task 2: PID with anti-windup

**Files:**
- Create: `src/control/mod.rs`, `src/control/pid.rs`
- Test: `src/control/pid.rs` 내 `#[cfg(test)]`

**Interfaces:**
- Produces: `pub struct Pid { /* kp, ki, kd, integral, prev_err, i_min, i_max */ }`, `impl Pid { pub fn new(kp: f32, ki: f32, kd: f32, i_min: f32, i_max: f32) -> Self; pub fn step(&mut self, err: f32, dt: f32) -> f32; pub fn reset(&mut self); }` (미분은 오차 미분, `prev_err` 사용)

- [ ] **Step 1: 실패 테스트 작성**

테스트 3개: (a) P-only 게인 2.0, 오차 1.0 → 출력 2.0. (b) I 동작: ki=1, 오차 1을 0.001초 × 1000스텝 → 출력이 시간에 비례해 증가. (c) anti-windup: `i_min=-1, i_max=1`로 두고 포화 조건 재현 — 큰 오차를 오래 넣어도 적분항이 `i_max`를 넘지 않음. (d) `reset` 후 적분·prev_err 초기화.

- [ ] **Step 2: 테스트 실패 확인**

Run: `cargo test control::pid`
Expected: FAIL (Pid 미정의)

- [ ] **Step 3: 구현** — 조건부 적분: 이번 스텝 출력이 이미 포화 방향이고 오차가 같은 부호면 적분 중지.

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test control::pid`
Expected: PASS

- [ ] **Step 5: Commit** — `git commit -m "feat: PID controller with conditional anti-windup"`

---

### Task 3: 모터 모델 (지연 라인 + 1차 지연)

**Files:**
- Create: `src/motor.rs`
- Test: `src/motor.rs` 내 `#[cfg(test)]`

**Interfaces:**
- Consumes: 없음 (독립)
- Produces: `pub struct Motor { delay_line: VecDeque<f32>, delay_steps: usize, state: f32, tau_s: f32, dt: f32 }`; `impl Motor { pub fn new(delay_s: f32, tau_s: f32, dt: f32) -> Self; pub fn set_delay(&mut self, delay_s: f32); pub fn step(&mut self, command: f32) -> f32 /* 정규화 회전속도 ω 0..1 */; pub fn omega(&self) -> f32; pub fn reset(&mut self); }`; `pub fn thrust(omega: f32, tmax: f32) -> f32 /* k·ω², 클램프 [0, tmax] */`

- [ ] **Step 1: 실패 테스트 작성**

테스트 4개: (a) 순수 지연: `tau=0, delay=0.05`(=50스텝)에서 스텝 입력 → 정확히 50스텝까지 출력 0, 51스텝부터 응답. (b) 1차 지연: `delay=0, tau=0.1`에서 스텝 입력 후 시상수 시간(0.1 s)에 출력 ≈ `1 - e^{-1}` ≈ 0.632 (허용오차 0.02). (c) **zero-delay**: `delay=0`이 지연 라인 없는 것과 동일 결과. (d) `set_delay`로 지연 변경 시 상태 보존 (버퍼 리사이즈, 실행 중 슬라이더 변경 반영). (e) `thrust`: ω=0.5, tmax=12 → 3.0; ω 클램프 확인.

- [ ] **Step 2: 테스트 실패 확인**

Run: `cargo test motor`
Expected: FAIL

- [ ] **Step 3: 구현** — `delay_line`은 최근 명령 히스토리, `step`은 맨 뒤 값 꺼내 1차 필터 `state += (delayed - state) * dt/tau.max(dt)`.

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test motor`
Expected: PASS

- [ ] **Step 5: Commit** — `git commit -m "feat: motor model with transport delay and first-order lag"`

---

### Task 4: 바람 생성기

**Files:**
- Create: `src/wind.rs`
- Test: `src/wind.rs` 내 `#[cfg(test)]`

**Interfaces:**
- Produces: `pub enum GustKind { None, Step, Sine, SmoothRandom }`, `pub struct WindGen { /* base: Vector3<f32>, gust_kind, gust_amplitude: f32, gust_period_s: f32, t: f32, state: f32 */ }`; `impl WindGen { pub fn new() -> Self; pub fn set(&mut self, base: Vector3<f32>, kind: GustKind, amplitude: f32, period_s: f32); pub fn step(&mut self, dt: f32) -> Vector3<f32>; pub fn pulse(&mut self, amplitude: f32, duration_s: f32) /* UI 버튼용: 임시 스텝 돌풍 */; pub fn reset(&mut self); }`

- [ ] **Step 1: 실패 테스트 작성**

테스트: (a) 기본풍만: `gust=None` → 항상 base 반환. (b) Step: 시작 후 `period/2` 동안 `base + amplitude`, 이후 base 복귀. (c) Sine: 진폭·주기 확인 (반주기에서 최대). (d) `pulse`는 `t` 구간 동안 추가 돌풍. (e) SmoothRandom: 연속 샘플 간 점프가 진폭 이하 (1차 마르코프).

- [ ] **Step 2: 실패 확인** — `cargo test wind` → FAIL
- [ ] **Step 3: 구현**
- [ ] **Step 4: 통과 확인** — `cargo test wind` → PASS
- [ ] **Step 5: Commit** — `git commit -m "feat: wind generator with steady, gust, and pulse modes"`

---

### Task 5: 기체 강체동역학

**Files:**
- Create: `src/drone.rs`
- Test: `src/drone.rs` 내 `#[cfg(test)]`

**Interfaces:**
- Consumes: 없음
- Produces: `pub struct DroneParams { pub mass: f32, pub arm_length: f32, pub inertia: [f32; 3] /* Ixx, Iyy, Izz */, pub tmax: f32, pub drag_lin: f32, pub drag_quad: f32, pub ang_damp: f32, pub yaw_tau_coef: f32 }` (+ `Default` = 650 g, arm 0.15 m, I=[0.02,0.02,0.04], Tmax=12.7 N ≈ 2:1 추력중비)
- Produces: `pub struct DroneState { pub pos: Vector3<f32>, pub vel: Vector3<f32>, pub quat: UnitQuaternion<f32>, pub omega_body: Vector3<f32> }`
- Produces: `pub struct Drone { pub state: DroneState }`; `impl Drone { pub fn new() -> Self; pub fn step(&mut self, omegas: [f32; 4], wind: Vector3<f32>, p: &DroneParams, dt: f32) }`
  - 모터 인덱스 규약: `[FL, FR, RL, RR]`, 기준축 x=전방, y=좌측, z=상방. 모터 추력은 기체 z축 방향. 믹싱에 쓰는 토크: `τ_pitch = arm·(T_FL+T_FR) − arm·(T_RL+T_RR)`, `τ_roll = arm·(T_RL+T_FL) − arm·(T_FR+T_RR)` 부호는 UI에 축 정의 표시 (Task 9).
  - 적분: 준-암시적 오일러. 저항은 상대풍(`vel − wind`)에 선형+제곱 항.

- [ ] **Step 1: 실패 테스트 작성**

테스트 4개: (a) 자유낙하: 모터 0, 중력 가속 확인 (1초 후 |vel| ≈ 9.8, 허용오차 0.3). (b) 호버 추력: 4모터 `ω = sqrt(m·g/(4·Tmax))`로 수평 자세 → 1초 후 z 속도 |·| < 0.05 m/s. (c) 차동 추력 → 롤: FL/RL 최대, FR/RR 0 → roll이 양(또는 음, 규약 일관) 방향으로 증가. (d) 바람 작용: 기본풍 5 m/s에 정지 기체 → 저항으로 풍방향 가속, 상대풍 계산 확인.

- [ ] **Step 2: 실패 확인** — `cargo test drone` → FAIL
- [ ] **Step 3: 구현** — 쿼터니온 갱신은 body rates로 `UnitQuaternion::new` 소각회전 합성 또는 `quaternion` 미분 적분 (nalgebra 기능 사용).
- [ ] **Step 4: 통과 확인** — `cargo test drone` → PASS
- [ ] **Step 5: Commit** — `git commit -m "feat: quadrotor rigid-body dynamics"`

---

### Task 6: 캐스케이드 컨트롤러

**Files:**
- Create: `src/control/cascade.rs` (`src/control/mod.rs`에 trait 추가)
- Test: `src/control/cascade.rs` 내 `#[cfg(test)]`

**Interfaces:**
- Consumes: `Pid` (Task 2), `DroneState` (Task 5), `Params` 내 게인 (Task 8까지는 로컬 `Gains` 구조 사용 후 Task 8에서 통합)
- Produces: `pub struct Target { pub pos: Vector3<f32>, pub yaw: f32 }`
- Produces: `pub trait Controller { fn step(&mut self, state: &DroneState, target: &Target, params: &Params, dt: f32) -> [f32; 4]; fn reset(&mut self); }`
- Produces: `pub struct CascadeController { /* 위치 P→속도 PID(3축) → 수직: 콜렉티브 추력, 수평: 목표 roll/pitch → 자세 PID(roll/pitch/yaw 각) → 목표 각속도 → 각속도 PID(3축, 토크 출력) → 믹싱 행렬 MIX: [[f32;4];4] 상수 */ }`; `impl Controller for CascadeController`
  - 수평 가속 명령 → 목표 기울기: `pitch_des = atan2(a_x_cmd, g)`, `roll_des = atan2(-a_y_cmd, g)` (부호는 축 규약에 맞게, 구현 시 UI 표시와 일치)
  - 콜렉티브 추력 = `m·(g + a_z_cmd)`를 기체 z축 방향으로, 믹싱 시 roll/pitch/yaw 토크와 합산

- [ ] **Step 1: 실패 테스트 작성**

통합 테스트 — `Drone` + `[Motor; 4]` + `CascadeController`를 dt=0.001로 직접 굴림 (Sim은 아직 없음, Task 7에서 재사용):
(a) **호버 + 일정 바람**: 지연 20 ms, 시상수 50 ms, 바람 x방향 3 m/s, 시작 위치 = 목표 → 10 s 후 수평 오차 < 0.05 m, z 오차 < 0.05 m (적분 항이 바람 상쇄 — 스펙 핵심).
(b) **수직 스텝**: t=2 s에 목표 +2 m → t=8 s에 오차 < 0.1 m, 오버슈트 < 0.3 m.
(c) **수평 스텝**: t=2 s에 목표 x +3 m → t=10 s에 오차 < 0.1 m.
(d) 게인이 0이면 자유낙하 (제어기가 실제로 연결됐음을 역증명).

- [ ] **Step 2: 실패 확인** — `cargo test control::cascade` → FAIL
- [ ] **Step 3: 구현** — 축별 `Pid` 인스턴스 + `MIX` 상수. 게인 기본값은 `Default for Gains`에 두고, 테스트가 (a)–(c)를 통과하도록 튜닝 (지연 20 ms에서 안정이 기준).
- [ ] **Step 4: 통과 확인** — `cargo test control::cascade` → PASS
- [ ] **Step 5: Commit** — `git commit -m "feat: cascaded position/attitude PID controller with mixing"`

---

### Task 7: 시뮬레이터 코어

**Files:**
- Create: `src/sim.rs`
- Test: `src/sim.rs` 내 `#[cfg(test)]`

**Interfaces:**
- Consumes: `Drone`, `Motor`, `WindGen`, `Controller` trait, `Params`
- Produces: `pub enum Mission { Hover, VerticalStep, HorizontalStep }`
- Produces: `pub struct History { pub t: Vec<f32>, pub pos_err: Vec<f32>, pub pos: Vec<[f32;3]>, pub roll_pitch_yaw: Vec<[f32;3]>, pub motors: Vec<[f32;4]>, pub wind: Vec<[f32;3]> }` (고정 스텝마다 push, UI에서 다운샘플)
- Produces: `pub struct Sim { pub params: Params, drone: Drone, motors: [Motor; 4], wind: WindGen, controller: Box<dyn Controller>, pub history: History, pub t: f32, pub diverged: bool, pub mission: Mission }`
- Produces: `impl Sim { pub fn new(params: Params) -> Self; pub fn apply_params(&mut self, params: Params) /* 리셋 없이 live 적용: 지연 변경은 set_delay, 나머지는 다음 스텝부터 */; pub fn step_frame(&mut self, real_dt: f32, time_scale: f32); pub fn set_mission(&mut self, m: Mission); pub fn wind_pulse(&mut self); pub fn reset(&mut self) }`
  - `step_frame`: `real_dt * time_scale`을 `DT` 스텝 수로 환산해 실행 (프레임당 최대 200스텝 가드)
  - 미션: `Hover` = 초기 위치 유지, `VerticalStep`/`HorizontalStep` = t=2 s에 목표 z / x +스텝 (크기는 Params `sim` 그룹에, 기본 2 m / 3 m)
  - 발산 감지: `|pos| > 1000 m` 또는 비유한수 → `diverged = true`, 스텝 중단

- [ ] **Step 1: 실패 테스트 작성**

테스트 4개: (a) `step_frame(1/60, 1.0)`을 600프레임 → t ≈ 10 s, 호버 유지 (기본 파라미터). (b) 발산: 게인 100배 → `diverged == true`, history에 NaN 없음. (c) 극단 파라미터(지연 0.3 s, 시상수 0.5 s, 게인 최대)에서도 NaN 없음. (d) `apply_params`로 지연 변경 후 `Motor.delay_steps` 일치, 실행 상태 유지 (pos가 리셋되지 않음). (e) `set_mission(HorizontalStep)` 후 스텝 시각 이후 목표가 변경됨.

- [ ] **Step 2: 실패 확인** — `cargo test sim` → FAIL
- [ ] **Step 3: 구현**
- [ ] **Step 4: 통과 확인** — `cargo test sim` → PASS
- [ ] **Step 5: Commit** — `git commit -m "feat: fixed-step simulation core with missions, divergence guard, live params"`

---

### Task 8: 파라미터·TOML·프리셋

**Files:**
- Create: `src/presets.rs`
- Modify: `src/params.rs` (전체 채움), `src/control/cascade.rs` (로컬 Gains → Params 게인으로 교체), `src/sim.rs` (필요 시 시그니처 정리)
- Test: `src/params.rs`, `src/presets.rs` 내 `#[cfg(test)]`

**Interfaces:**
- Consumes: Task 2–7의 모든 구조체
- Produces: `pub struct Params { pub drone: DroneParams, pub motor: MotorParams { delay_s, tau_s }, pub gains: Gains, pub wind: WindParams { base: [f32;3], gust: GustKind, amplitude, period_s }, pub sim: SimParams { step_height_m, step_dist_m, step_time_s } }` — 전부 `Serialize, Deserialize, Clone`, TOML 저장/로드: `Params::to_toml(&self) -> String`, `Params::from_toml(&str) -> Result<Params, ...>`
- Produces: `pub enum Preset { Tuned, DelayHeavy, SlowMotor, StrongWind, Unstable }`; `pub fn preset_params(p: Preset) -> Params` (Unstable = 기본 게인 ×3 + 지연 100 ms — 실행하면 진동·발산하는 교육용 프리셋)
- Produces: `Gains` 기본값 = Task 6 테스트에서 통과한 튜닝값 (호버 + 바람 3 m/s 상쇄 검증 완료 상태)

- [ ] **Step 1: 실패 테스트 작성** — (a) TOML 왕복: `from_toml(to_toml(p)) == p`. (b) 각 프리셋이 서로 다른 지연/게인 값을 가짐. (c) Tuned 프리셋으로 Task 6 (a) 시나리오 재실행 → 통과 (게인이 Params로 옮겨간 후에도).
- [ ] **Step 2: 실패 확인** — `cargo test params presets control::cascade` → FAIL
- [ ] **Step 3: 구현**
- [ ] **Step 4: 통과 확인** — 전부 PASS
- [ ] **Step 5: Commit** — `git commit -m "feat: TOML params with presets; controller reads live Params"`

---

### Task 9: GUI 셸 (eframe + 슬라이더)

**Files:**
- Create: `src/app.rs`, `src/viz.rs` (빈 플레이스홀더), `src/main.rs` 본체
- Test: 없음 (GUI) — 컴파일 + 실행 확인

**Interfaces:**
- Consumes: `Sim`, `Params`, `Preset`
- Produces: `pub struct DroneApp { sim: Sim, draft: Params, paused: bool }`; `impl eframe::App for DroneApp`
  - 좌측 패널: 슬라이더 그룹 4개 (PID 게인 / 모터·지연 / 바람 / 기체 스펙) — **`draft`를 수정하고 "적용" 없이 즉시 `sim.apply_params(draft.clone())`** (Review Focus 4)
  - 상단 바: 미션 콤보(Hover/수직/수평), 바람 펄스 버튼, 일시정지/리셋, 배속 슬라이더(0.1–5×), 프리셋 드롭다운(5종, 한국어 라벨: 기준 튜닝 / 지연 과다 / 느린 모터 / 강풍 / 불안정)
  - 지연 시상수 등 슬라이더 범위: 지연 0–300 ms, 시상수 0–500 ms (스펙 §7.3)
  - `diverged == true`면 중앙에 "발산 — 리셋하세요" 오버레이 (Review Focus 1)
  - 믹싱/축 규약 텍스트를 좌측 패널 하단에 표시 (스펙 §8)

- [ ] **Step 1: `main.rs`에 eframe 실행 + `DroneApp` 구현**
- [ ] **Step 2: 컴파일 확인** — `cargo build` → OK
- [ ] **Step 3: 실행 확인** — `cargo run --release`로 창이 뜨고, 슬라이더를 움직이면 시뮬레이션이 리셋 없이 반응하는지, 프리셋 전환·일시정지·리셋이 동작하는지 수동 확인 (체크리스트 각 항목)
- [ ] **Step 4: 전체 테스트 회귀** — `cargo test` → PASS
- [ ] **Step 5: Commit** — `git commit -m "feat: egui shell with live parameter sliders, presets, missions"`

---

### Task 10: 2D 시각화

**Files:**
- Modify: `src/viz.rs`
- Test: 없음 (렌더링) — 컴파일 + 실행 확인

**Interfaces:**
- Consumes: `Sim.history`, `DroneState`, `Target`
- Produces: `pub fn draw_central(ui: &mut egui::Ui, sim: &Sim, view: View)`; `pub enum View { Side, Top }`
  - 측면 뷰: x–z 평면, 기체를 사각형+기울기로 표시(roll/pitch 화살표), 목표점 마커, 궤적(최근 10 s)
  - 상단 뷰: x–y 평면 동일 구성
  - 바람 화살표 표시

- [ ] **Step 1: `viz.rs` 구현 후 Task 9의 중앙 패널에 연결** (플레이스홀더 교체)
- [ ] **Step 2: 컴파일** — `cargo build` → OK
- [ ] **Step 3: 실행 확인** — `cargo run --release`: 호버 시 기체가 목표점에 머무르고, 바람 펄스 시 밀렸다 복귀하는 것이 화면에서 보이는지 수동 확인
- [ ] **Step 4: 회귀** — `cargo test` → PASS
- [ ] **Step 5: Commit** — `git commit -m "feat: 2D side/top visualization with trajectory and wind arrow"`

---

### Task 11: 실시간 그래프

**Files:**
- Create: `src/plots.rs`
- Modify: `src/app.rs` (하단 패널 연결)
- Test: 없음 (렌더링) — 컴파일 + 실행 확인

**Interfaces:**
- Consumes: `Sim.history` (다운샘플: 최근 N초를 화면 폭에 맞게 추출 — 로직이면 `plots.rs`에서 순수 함수 `pub fn window(history: &History, seconds: f32) -> History`로 빼고 유닛 테스트)
- Produces: `pub fn draw_bottom(ui: &mut egui::Ui, sim: &Sim, tab: PlotTab)`; `pub enum PlotTab { PositionError, Attitude, MotorOutputs }` — 탭 3개, egui_plot 라인 플롯, 시간축 자동 스크롤

- [ ] **Step 1: `window` 순수 함수 실패 테스트** — 최근 N초만 잘라내는지, 스텝 수 0/1 경계. `cargo test plots` → FAIL → 구현 → PASS
- [ ] **Step 2: `draw_bottom` 구현, Task 9 하단에 연결**
- [ ] **Step 3: 컴파일** — `cargo build` → OK
- [ ] **Step 4: 실행 확인** — `cargo run --release`: 지연 프리셋에서 진동이 그래프에 나타나는지, 탭 전환이 되는지 수동 확인
- [ ] **Step 5: 회귀 + Commit** — `cargo test` PASS; `git commit -m "feat: real-time plots for position error, attitude, motor outputs"`

---

## Self-Review 결과

- **스펙 커버리지**: 물리(§7.3)=Task 5, 모터 지연(§7.3)=Task 3, 바람(§2)=Task 4, 캐스케이드 PID+anti-windup(§7.2)=Task 2/6, trait(§7.2)=Task 6, UI 구성(§7.4)=Task 9/10/11, 프리셋(§2)=Task 8, TOML(§2)=Task 8, 발산 가드(Review Focus)=Task 7. 누락 없음.
- **단계 검사**: 각 단계가 단일 행위 (테스트 작성/실행/구현/커밋). GUI 태스크는 유닛 테스트 대신 실행 확인 체크리스트.
- **타입 일관성**: `Params`는 Task 1 스모크 → Task 8 확정, `Controller` trait 시그니처 Task 6/7 일치 (`&Params` 인자), `Motor::set_delay` Task 3 정의 → Task 7 `apply_params` 사용.
- **Review Focus**: 5개 항목 모두 소유 태스크의 테스트로 고정됨 (위 각 줄 참조).
- **비례**: 물리·제어 알고리즘 본문은 시그니처+테스트로 결정 가능한 것만 기술, 알고리즘 설명은 최소화.
