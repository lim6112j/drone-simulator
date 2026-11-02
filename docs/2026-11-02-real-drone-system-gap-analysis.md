# 실제 드론 시스템 개발을 위해 더 필요한 것 — 갭 분석

날짜: 2026-11-02
상태: 참고용 (미구현, 로드맵 후보)
근거: 이 저장소의 현재 코드 + 온보딩 리뷰 (`src/` 전체, 41 테스트 통과 확인)

---

## 0. 요약

현재 저장소는 **내부 루프(위치→속도→자세→각속도→믹싱)의 교과서적 모델**이다.
실제 드론 시스템은 여기에 **센서 · 상태추정 · 실제 구동기 · 통신 · 안전장치 · 검증 도구**가 더해진다.

가장 값싸고 임팩트 큰 순서:

1. 센서 모델 + 상태추정(EKF/상보필터)
2. 실제 모터/ESC 모델 (추력 곡선 표, 전압 강하, RPM 거버너)
3. SITL/HIL 브리지 + MAVLink
4. 비행 로그 포맷 + 리플레이 + 튜닝 스윕
5. Failsafe ·arming · 비행 모드
6. companion computer 인터페이스

---

## 1. 물리 충실도 갭

| 빠진 것 | 왜 중요한가 | 현재 코드 |
|---|---|---|
| **센서 노이즈/바이어스/드리프트/진동** | 컨트롤러가 현재 **ground truth**를 본다. 실제 FC는 진공 상태를 절대 모른다. 자이로 노이즈는 D항으로 에일리어싱되고, 진동은 모터 desync를 만든다 | `src/sim.rs:161` `state`를 clone해서 그대로 `controller.step` |
| **상태추정 (EKF)** | 자세/속도/위치는 IMU+기압+GPS+ToF/optical flow를 **융합해 추정**한다. 실무 튜닝 고통의 대부분이 여기 | 추정기 자체가 없음 — 피드백이 진실값에 직접 걸림 |
| **자이로스코픽 커플링 `ω × Iω`** | 공격적인 roll/flip에서 축이 서로 묶인다. 축별 관성으로는 안 나온다 | `src/drone.rs:80` 토크를 축별로 독립적으로 나눔 |
| **추력 곡선 표** | 실제 추력 vs RPM은 `k·ω²`가 아니다. 저/고 RPM에서 프로펠러 효율이 떨어진다 | `src/motor.rs:71` `thrust = tmax·ω²` |
| **모터 반작용 토크 / 항력 토크** | 요 축은 현재 **명령된 차동**만 존재해서 맞서는 힘이 없다 | `src/drone.rs:74` 단일 `yaw_tau_coef` |
| **배터리 전압 강하** | 부하 시 추력이 10~20% 떨어져 **시변 플랜트**가 된다 | 전기 모델 없음, `tmax` 상수 |
| **공력 비등방성 · 지면효과 · 프로펠러 워시** | 지면 근처 호버, 벽 근처, 수평이동 항력 차이 | `src/drone.rs:92` 등방성 drag, 지면 없음 |
| **비균일 풍장** | 실제 돌풍은 공간적으로 상관된 스펙트럼(von Kármán/Dryden), 전역 벡터 하나가 아니다 | `src/wind.rs` 스텝당 벡터 1개 반환 |
| **진동 / 구조적 공진** | 프레임 공진이 실무 튜닝 1위 문제 (dynamic notch filter가 존재하는 이유) | 없음 |

---

## 2. 비행 소프트웨어 갭

- **레이트 루프 주기**: 실제 FC는 4~8 kHz에서 결정론적 스케줄링으로 자세/레이트 루프를 돈다. 이 시뮬은 1 kHz이며 한 프레임에 스텝을 최대 200개 몰아넣는다 (`src/sim.rs:147`).
- **필터**: 자이로 로우패스 + static/dynamic notch, D항 로우패스. 노이즈를 모델링하지 않으면 필터는 의미가 없다.
- **arming 상태 기계, 비행 모드, failsafe**: RC loss → RTH, 저전압 → 착륙, geofence, 고도 상한. 전혀 없음.
- **통신/프로토콜**: MAVLink (PX4/ArduPilot), CRTP (Betaflight) 또는 자체 링크; RC 입력 (CRSF/ELRS), 텔레메트리, OSD.
- **companion computer 경로**: offboard 제어, VIO/SLAM, 장애물 회피, 미션 플래닝 (기본은 ROS 2).

---

## 3. 도구 체인 갭 (물리 갭보다 싸고 임팩트 큼)

- **SITL / HIL**: **실제 펌웨어**(PX4, ArduPilot)를 내 물리와 붙여 돌린다. PX4와 ArduPilot 둘 다 네이티브 SITL을 제공하고, HITL도 지원한다. 이 저장소의 `Drone::step`는 그대로 FDM 후보 — 재사용 가치가 가장 높은 지점.
- **비행 로그**: Betaflight blackbox / PX4 `.ulg`가 루프 1회분의 자이로·setpoint·P/I/D·모터출력을 기록한다. 현재 `History` 구조는 모양은 맞지만 **표준 포맷이 없고, 1 kHz라 무한 증가**한다.
- **시스템 식별**: 스러스트 스탠드(힘 vs ESC 명령), 스텝 응답(지연/τ), 바이필러 진자(관성), 로그 피팅(항력/추력 계수). ArduPilot에는 sysid 도구체인이 내장되어 있다. `examples/*.toml`은 **종합한 대표값**이며, 실제 수치는 이 측정들에서 나온다.
- **자동 튜닝 + 몬테카를로**: 게인/지연을 수백 개 조합으로 스윕해서 안정 여유를 플롯한다. 발산 가드 + 고정 스텝 결정론 덕에 이 저장소에서 쉽게 할 수 있지만 아무도 안 만들어 뒀다.
- **SIL 회귀 / CI**: 현재 41개 테스트는 유닛 수준. 시나리오 수준 인수 테스트("5 m/s 돌풍에서 30초간 호버 오차 5 cm 이내")가 추가되어야 한다.

---

## 4. 프로세스 · 안전 (비코드)

props-off 테스트 리그, 스러스트 스탠드 벤치, 시험 장소, 추락 포렌식, 리스크 평가, 규제(Remote ID, Part 107 / EASA 카테고리), 기체별 설정 버전 관리, 보험. 소프트웨어는 아니지만 "실제 드론 시스템 개발"에 포함된다.

---

## 5. 단계별 로드맵

| 단계 | 내용 | 이 저장소에서의 착수점 | 난이도 |
|---|---|---|---|
| 1 | 센서 모델(노이즈/바이어스/진동) + 상보필터 또는 EKF | `sim.rs`의 `state` 직접 피드백을 센서+추정기 뒤로 밀기 | 중 |
| 2 | 실제 모터/ESC: 추력 곡선 표, RPM 거버너, 전압 강하, 양방향 DShot RPM | `motor.rs` (현재 1차 지연 + ω²) | 중 |
| 3 | SITL/HIL 브리지 + MAVLink | `Drone::step`를 FDM으로 노출 | 상 |
| 4 | 비행 로그 포맷 + 리플레이 + 튜닝 스윕 | `History`에 포맷/트리밍 추가, `presets.rs`를 스윕 harness로 | 중 |
| 5 | arming / 비행 모드 / failsafe | 신규 모듈 | 중 |
| 6 | companion computer 인터페이스 (offboard API) | 신규 모듈 | 상 |

1~2는 이 저장소 안에서 자급자족한다. 3부터는 실제 펌웨어 작업을 위한 인프라가 된다.

---

## 6. 검증 방법 (모델을 추가할 때마다)

| 추가하는 모델 | 검증 수단 |
|---|---|
| 모터 지연 / 시상수 | 스텝 응답 측정 (스러스트 스탠드 + 오실로스코프), 63% 도달 시간 |
| 추력 곡선 | 스러스트 스탠드: ESC 명령 → 힘 표 |
| 관성 | 바이필러 진자 또는 `I ≈ (1/12)·m·L²` 근사 |
| 항력 | 풍동 데이터 또는 풍압등급 역산, 논문값 |
| 센서 노이즈 | 정지 상태 분산, 알란 편차 |
| EKF | ground-truth 대비 위치/자세 오차, 혁신 시퀀스 잔차 |

---

## 7. 참고 자료

- PX4 Simulation (SITL/HITL): https://docs.px4.io/main/en/simulation/
- ArduPilot SITL Simulator: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- ArduPilot System Identification: https://ardupilot.org/copter/docs/systemid-model-development.html
- Betaflight Blackbox logging: https://betaflight.com/docs/wiki/guides/current/Black-Box-logging-and-usage
- Betaflight Blackbox 사용 가이드: https://oscarliang.com/blackbox/
- Data-Driven System Identification of Quadrotors (arXiv 2404.07837): https://arxiv.org/html/2404.07837v2
