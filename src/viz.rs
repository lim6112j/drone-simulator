//! 2D 시각화: 측면(x–z) / 상단(x–y) 뷰, 궤적·목표점·바람 화살표.
use crate::sim::Sim;
use eframe::egui::{self, emath::Rot2, Color32, Pos2, Rect, Stroke, Vec2};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum View {
    Side,
    Top,
}

const PX_PER_M: f32 = 60.0;
const TRAIL_S: f32 = 10.0;

/// 세계 좌표 쌍 → 화면 좌표. up_axis는 위쪽 양의 방향 성분.
fn world_to_screen(rect: Rect, origin: Pos2, horizontal: f32, vertical: f32) -> Pos2 {
    Pos2::new(
        origin.x + horizontal * PX_PER_M,
        origin.y - vertical * PX_PER_M,
    )
}

fn draw_grid(p: &egui::Painter, rect: Rect, origin: Pos2) {
    let grid = Color32::from_rgb(50, 52, 58);
    let mut x = 0.0;
    while origin.x + x * PX_PER_M < rect.right() {
        let px = origin.x + x * PX_PER_M;
        p.line_segment([Pos2::new(px, rect.top()), Pos2::new(px, rect.bottom())], Stroke::new(1.0, grid));
        let px2 = origin.x - x * PX_PER_M;
        if x > 0.0 {
            p.line_segment([Pos2::new(px2, rect.top()), Pos2::new(px2, rect.bottom())], Stroke::new(1.0, grid));
        }
        x += 1.0;
    }
    let mut y = 0.0;
    while origin.y - y * PX_PER_M > rect.top() {
        let py = origin.y - y * PX_PER_M;
        p.line_segment([Pos2::new(rect.left(), py), Pos2::new(rect.right(), py)], Stroke::new(1.0, grid));
        let py2 = origin.y + y * PX_PER_M;
        if y > 0.0 {
            p.line_segment([Pos2::new(rect.left(), py2), Pos2::new(rect.right(), py2)], Stroke::new(1.0, grid));
        }
        y += 1.0;
    }
}

pub fn draw_central(ui: &mut egui::Ui, sim: &Sim, view: View) {
    let (rect, _resp) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 0.0, Color32::from_rgb(26, 27, 31));

    let state = sim.state();
    let pos = state.pos;
    let target = sim.target_pos();

    // 뷰별 좌표 성분: (가로, 세로)
    let (h_pos, v_pos, h_t, v_t, angle) = match view {
        View::Side => (pos.x, pos.z, target.x, target.z, state.quat.euler_angles().1),
        View::Top => (pos.x, pos.y, target.x, target.y, state.quat.euler_angles().2),
    };

    // 화면 원점: 목표점을 화면 중앙보다 약간 아래에 배치 (수직 스텝 시 위로 갈 공간)
    let origin = Pos2::new(rect.center().x, rect.center().y + rect.height() * 0.25);
    draw_grid(&p, rect, origin);

    // 궤적 (최근 10 s, 다운샘플)
    let t0 = (sim.t - TRAIL_S).max(0.0);
    let h = &sim.history;
    let start = h.t.iter().position(|&t| t >= t0).unwrap_or(0);
    let n = h.t.len();
    if n > 1 {
        let step = (n / 600).max(1);
        let pts: Vec<Pos2> = (start..n)
            .step_by(step)
            .map(|i| {
                let [x, y, z] = h.pos[i];
                let (hh, vv) = match view {
                    View::Side => (x, z),
                    View::Top => (x, y),
                };
                world_to_screen(rect, origin, hh, vv)
            })
            .collect();
        p.add(egui::Shape::line(pts, Stroke::new(1.5, Color32::from_rgb(80, 140, 220))));
    }

    // 목표점 마커 (십자 + 원)
    let tp = world_to_screen(rect, origin, h_t, v_t);
    let cross = Stroke::new(1.5, Color32::from_rgb(240, 200, 80));
    p.line_segment([Pos2::new(tp.x - 8.0, tp.y), Pos2::new(tp.x + 8.0, tp.y)], cross);
    p.line_segment([Pos2::new(tp.x, tp.y - 8.0), Pos2::new(tp.x, tp.y + 8.0)], cross);
    p.circle_stroke(tp, 12.0, cross);

    // 기체: 0.3 m 사각형 + 앞부분 방향 표시
    let dp = world_to_screen(rect, origin, h_pos, v_pos);
    let rot = Rot2::from_angle(angle);
    let half = 0.15 * PX_PER_M; // 사각형 반변
    let corners_local = [
        Vec2::new(-half, -half * 0.4),
        Vec2::new(half, -half * 0.4),
        Vec2::new(half, half * 0.4),
        Vec2::new(-half, half * 0.4),
    ];
    let corners: Vec<Pos2> = corners_local
        .iter()
        .map(|c| {
            let r = rot * *c;
            Pos2::new(dp.x + r.x, dp.y + r.y)
        })
        .collect();
    p.add(egui::Shape::convex_polygon(
        corners,
        Color32::from_rgb(70, 130, 200),
        Stroke::new(1.0, Color32::from_rgb(150, 190, 240)),
    ));
    // 기수 표시
    let nose = rot * Vec2::new(half + 8.0, 0.0);
    p.line_segment(
        [dp, Pos2::new(dp.x + nose.x, dp.y + nose.y)],
        Stroke::new(2.0, Color32::from_rgb(230, 240, 250)),
    );

    // 바람 화살표 (좌측 하단 고정)
    if let Some(&[wx, wy, _]) = h.wind.last() {
        let (wh, wv) = match view {
            View::Side => (wx, 0.0),
            View::Top => (wx, wy),
        };
        let base = Pos2::new(rect.left() + 40.0, rect.bottom() - 40.0);
        let tip = base + Vec2::new(wh * 6.0, -wv * 6.0);
        let dir = tip - base;
        if dir.length() > 4.0 {
            p.arrow(base, tip - base, Stroke::new(2.0, Color32::from_rgb(120, 200, 220)));
        }
        ui.painter_at(rect).text(
            Pos2::new(rect.left() + 16.0, rect.bottom() - 24.0),
            egui::Align2::LEFT_TOP,
            format!("바람 {wx:.1} m/s"),
            egui::FontId::proportional(13.0),
            Color32::from_rgb(120, 200, 220),
        );
    }

    // HUD
    let err = h.pos_err.last().copied().unwrap_or(0.0);
    let hud = format!(
        "t={:.1}s  pos=({:.2}, {:.2}, {:.2}) m  오차={:.3} m",
        sim.t, pos.x, pos.y, pos.z, err
    );
    p.text(
        Pos2::new(rect.left() + 16.0, rect.top() + 12.0),
        egui::Align2::LEFT_TOP,
        hud,
        egui::FontId::proportional(15.0),
        Color32::from_rgb(200, 205, 215),
    );
}
