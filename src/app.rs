//! egui 프론트엔드: 좌측 파라미터 패널, 상단 컨트롤 바, 중앙 2D 시각화, 하단 그래프.
use crate::params::Params;
use crate::plots::{self, PlotTab};
use crate::presets::{preset_label, preset_params, ALL_PRESETS, Preset};
use crate::sim::{Mission, Sim};
use crate::viz::{self, View};
use eframe::egui;

pub struct DroneApp {
    pub sim: Sim,
    pub draft: Params,
    pub paused: bool,
    pub time_scale: f32,
    pub view: View,
    pub plot_tab: PlotTab,
    pub selected_preset: Option<Preset>,
}

impl Default for DroneApp {
    fn default() -> Self {
        Self::new()
    }
}

impl DroneApp {
    pub fn new() -> Self {
        let draft = Params::default();
        Self {
            sim: Sim::new(draft.clone()),
            draft,
            paused: false,
            time_scale: 1.0,
            view: View::Side,
            plot_tab: PlotTab::PositionError,
            selected_preset: Some(Preset::Tuned),
        }
    }

    fn params_panel(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("PID 게인");
            let g = &mut self.draft.gains;
            ui.add(egui::Slider::new(&mut g.pos_kp, 0.0..=10.0).text("위치 P"));
            ui.add(egui::Slider::new(&mut g.vel_kp, 0.0..=10.0).text("속도 P"));
            ui.add(egui::Slider::new(&mut g.vel_ki, 0.0..=5.0).text("속도 I (바람 상쇄)"));
            ui.add(egui::Slider::new(&mut g.vel_kd, 0.0..=5.0).text("속도 D"));
            ui.add(egui::Slider::new(&mut g.att_kp, 0.0..=15.0).text("자세 P"));
            ui.add(egui::Slider::new(&mut g.rate_kp, 0.0..=3.0).text("각속도 P"));
            ui.add(egui::Slider::new(&mut g.rate_ki, 0.0..=5.0).text("각속도 I"));
            ui.add(egui::Slider::new(&mut g.yaw_att_kp, 0.0..=15.0).text("요 자세 P"));
            ui.add(egui::Slider::new(&mut g.yaw_rate_kp, 0.0..=3.0).text("요 각속도 P"));
            ui.add(egui::Slider::new(&mut g.vmax, 0.5..=8.0).text("최대 속도 (m/s)"));
            ui.add(egui::Slider::new(&mut g.amax, 0.5..=15.0).text("최대 가속 (m/s²)"));
            ui.add(egui::Slider::new(&mut g.tilt_max_deg, 5.0..=60.0).text("최대 기울기 (°)"));

            ui.separator();
            ui.heading("모터 · 지연");
            let m = &mut self.draft.motor;
            ui.add(
                egui::Slider::new(&mut m.delay_s, 0.0..=0.3)
                    .text("순수 지연 (s)")
                    .suffix(" s"),
            );
            ui.add(
                egui::Slider::new(&mut m.tau_s, 0.005..=0.5)
                    .text("시상수 τ (s)")
                    .suffix(" s"),
            );

            ui.separator();
            ui.heading("바람");
            let w = &mut self.draft.wind;
            let mut mag = (w.base[0] * w.base[0] + w.base[1] * w.base[1]).sqrt();
            let mut dir = w.base[1].atan2(w.base[0]).to_degrees();
            if ui
                .add(egui::Slider::new(&mut mag, 0.0..=12.0).text("세기 (m/s)"))
                .changed()
                || ui
                    .add(egui::Slider::new(&mut dir, -180.0..=180.0).text("방향 (°)"))
                    .changed()
            {
                let r = dir.to_radians();
                w.base = [mag * r.cos(), mag * r.sin(), 0.0];
            }
            ui.horizontal(|ui| {
                ui.label("돌풍:");
                egui::ComboBox::from_id_salt("gust_kind")
                    .selected_text(match w.gust {
                        crate::wind::GustKind::None => "없음",
                        crate::wind::GustKind::Step => "스텝",
                        crate::wind::GustKind::Sine => "사인",
                        crate::wind::GustKind::SmoothRandom => "요동",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut w.gust, crate::wind::GustKind::None, "없음");
                        ui.selectable_value(&mut w.gust, crate::wind::GustKind::Step, "스텝");
                        ui.selectable_value(&mut w.gust, crate::wind::GustKind::Sine, "사인");
                        ui.selectable_value(
                            &mut w.gust,
                            crate::wind::GustKind::SmoothRandom,
                            "요동",
                        );
                    });
            });
            ui.add(egui::Slider::new(&mut w.amplitude, 0.0..=10.0).text("돌풍 진폭 (m/s)"));
            ui.add(egui::Slider::new(&mut w.period_s, 0.5..=10.0).text("돌풍 주기 (s)"));

            ui.separator();
            ui.heading("기체 스펙");
            let d = &mut self.draft.drone;
            ui.add(egui::Slider::new(&mut d.mass, 0.1..=3.0).text("질량 (kg)"));
            ui.add(egui::Slider::new(&mut d.tmax, 4.0..=30.0).text("최대 추력 (N)"));

            ui.separator();
            ui.heading("미션 스텝");
            let s = &mut self.draft.sim;
            ui.add(egui::Slider::new(&mut s.step_height_m, 0.5..=5.0).text("수직 스텝 (m)"));
            ui.add(egui::Slider::new(&mut s.step_dist_m, 0.5..=10.0).text("수평 스텝 (m)"));
            ui.add(egui::Slider::new(&mut s.step_time_s, 0.5..=5.0).text("스텝 시각 (s)"));

            ui.separator();
            ui.colored_label(
                egui::Color32::GRAY,
                "축 규약: x=전방, y=좌측, z=상방\n롤 +x(좌측 상승) · 피치 +y(기수 하강) · 요 +z(좌회전)\n모터 [FL, FR, RL, RR]",
            );
        });
    }
}

impl eframe::App for DroneApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // 상단 컨트롤 바
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("미션:");
                let mission = match self.sim.mission {
                    Mission::Hover => "호버",
                    Mission::VerticalStep => "수직 스텝",
                    Mission::HorizontalStep => "수평 스텝",
                };
                egui::ComboBox::from_id_salt("mission")
                    .selected_text(mission)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.sim.mission,
                            Mission::Hover,
                            "호버",
                        );
                        ui.selectable_value(
                            &mut self.sim.mission,
                            Mission::VerticalStep,
                            "수직 스텝",
                        );
                        ui.selectable_value(
                            &mut self.sim.mission,
                            Mission::HorizontalStep,
                            "수평 스텝",
                        );
                    });

                if ui.button("바람 펄스").clicked() {
                    self.sim.wind_pulse();
                }

                ui.separator();
                let pause_label = if self.paused { "▶ 재생" } else { "⏸ 일시정지" };
                if ui.button(pause_label).clicked() {
                    self.paused = !self.paused;
                }
                if ui.button("⟲ 리셋").clicked() {
                    self.sim.reset();
                }
                ui.add(
                    egui::Slider::new(&mut self.time_scale, 0.1..=5.0).text("배속"),
                );

                ui.separator();
                ui.label("프리셋:");
                let label = self.selected_preset.map(preset_label).unwrap_or("사용자 정의");
                egui::ComboBox::from_id_salt("preset")
                    .selected_text(label)
                    .show_ui(ui, |ui| {
                        for &p in &ALL_PRESETS {
                            if ui
                                .selectable_label(self.selected_preset == Some(p), preset_label(p))
                                .clicked()
                            {
                                self.selected_preset = Some(p);
                                self.draft = preset_params(p);
                            }
                        }
                    });
            });
        });

        // 좌측 파라미터 패널 — 슬라이더가 draft를 바꾸면 즉시 적용 (리셋 없음)
        egui::SidePanel::left("params").show(ctx, |ui| {
            self.params_panel(ui);
        });
        self.sim.apply_params(self.draft.clone());

        // 하단 그래프
        egui::TopBottomPanel::bottom("plots")
            .height_range(egui::Rangef::new(180.0, 320.0))
            .show(ctx, |ui| {
                plots::draw_bottom(ui, &self.sim, &mut self.plot_tab);
            });

        // 중앙 2D 시각화
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.view, View::Side, "측면");
                ui.selectable_value(&mut self.view, View::Top, "상단");
            });
            viz::draw_central(ui, &self.sim, self.view);

            if self.sim.diverged {
                egui::Area::new(egui::Id::new("diverge_overlay"))
                    .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                    .show(ctx, |ui| {
                        egui::Frame::default()
                            .fill(egui::Color32::from_rgba_unmultiplied(120, 0, 0, 220))
                            .inner_margin(16.0)
                            .show(ui, |ui| {
                                ui.colored_label(
                                    egui::Color32::WHITE,
                                    "발산 — 리셋하세요",
                                );
                            });
                    });
            }
        });

        // 시뮬레이션 진행
        if !self.paused && !self.sim.diverged {
            let dt = ctx.input(|i| i.stable_dt);
            self.sim.step_frame(dt, self.time_scale);
        }

        ctx.request_repaint();
    }
}
