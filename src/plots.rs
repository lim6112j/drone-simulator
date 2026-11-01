//! 실시간 그래프: 하단 패널 egui_plot 라인 플롯.
use crate::sim::{History, Sim};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlotTab {
    PositionError,
    Attitude,
    MotorOutputs,
}

/// history에서 최근 `seconds`초 구간만 잘라낸다 (시간축 자동 스크롤의 핵심).
pub fn window(history: &History, seconds: f32) -> History {
    let n = history.t.len();
    if n == 0 {
        return History::default();
    }
    let t_end = history.t[n - 1];
    let t0 = t_end - seconds;
    let start = history.t.iter().position(|&t| t >= t0).unwrap_or(n);
    let out = History {
        t: history.t[start..].to_vec(),
        pos_err: history.pos_err[start..].to_vec(),
        pos: history.pos[start..].to_vec(),
        roll_pitch_yaw: history.roll_pitch_yaw[start..].to_vec(),
        motors: history.motors[start..].to_vec(),
        wind: history.wind[start..].to_vec(),
    };
    out
}

pub fn draw_bottom(ui: &mut egui::Ui, sim: &Sim, tab: &mut PlotTab) {
    ui.horizontal(|ui| {
        ui.selectable_value(tab, PlotTab::PositionError, "위치 오차");
        ui.selectable_value(tab, PlotTab::Attitude, "자세");
        ui.selectable_value(tab, PlotTab::MotorOutputs, "모터 출력");
    });
    let h = window(&sim.history, 10.0);
    use egui_plot::{Line, Plot, PlotPoints};
    match tab {
        PlotTab::PositionError => {
            Plot::new("pos_err_plot")
                .legend(egui_plot::Legend::default())
                .y_axis_width(48)
                .show(ui, |plot_ui| {
                    let pts: PlotPoints = h
                        .t
                        .iter()
                        .zip(&h.pos_err)
                        .map(|(t, e)| [*t as f64, *e as f64])
                        .collect();
                    plot_ui.line(Line::new("위치 오차 (m)", pts));
                });
        }
        PlotTab::Attitude => {
            Plot::new("att_plot")
                .legend(egui_plot::Legend::default())
                .y_axis_width(48)
                .show(ui, |plot_ui| {
                    let n = h.t.len();
                    let line = |name: &str, idx: usize, rad: bool| {
                        let pts: PlotPoints = (0..n)
                            .map(|i| {
                                let v = h.roll_pitch_yaw[i][idx];
                                [*h.t.get(i).unwrap_or(&0.0) as f64, (if rad { v } else { v.to_degrees() }) as f64]
                            })
                            .collect();
                        Line::new(name, pts)
                    };
                    plot_ui.line(line("롤(°)", 0, false));
                    plot_ui.line(line("피치(°)", 1, false));
                    plot_ui.line(line("요(°)", 2, false));
                });
        }
        PlotTab::MotorOutputs => {
            Plot::new("motor_plot")
                .legend(egui_plot::Legend::default())
                .y_axis_width(48)
                .show(ui, |plot_ui| {
                    let n = h.t.len();
                    for (idx, name) in ["FL", "FR", "RL", "RR"].iter().enumerate() {
                        let pts: PlotPoints = (0..n)
                            .map(|i| {
                                let v = h.motors[i][idx];
                                [*h.t.get(i).unwrap_or(&0.0) as f64, v as f64]
                            })
                            .collect();
                        plot_ui.line(Line::new(*name, pts));
                    }
                });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hist_with_times(ts: &[f32]) -> History {
        let pos = ts.iter().map(|_| [0.0f32; 3]).collect();
        let att = ts.iter().map(|_| [0.0f32; 3]).collect();
        let motors = ts.iter().map(|_| [0.0f32; 4]).collect();
        let wind = ts.iter().map(|_| [0.0f32; 3]).collect();
        History {
            t: ts.to_vec(),
            pos_err: ts.iter().map(|t| *t).collect(),
            pos,
            roll_pitch_yaw: att,
            motors,
            wind,
        }
    }

    #[test]
    fn empty_history_gives_empty_window() {
        let w = window(&History::default(), 10.0);
        assert_eq!(w.t.len(), 0);
    }

    #[test]
    fn window_keeps_only_recent_seconds() {
        let h = hist_with_times(&[0.0, 1.0, 2.0, 3.0, 4.0, 5.0]);
        let w = window(&h, 2.0);
        assert_eq!(w.t, vec![3.0, 4.0, 5.0], "only t >= 3.0");
    }

    #[test]
    fn window_larger_than_history_returns_all() {
        let h = hist_with_times(&[0.0, 1.0, 2.0]);
        let w = window(&h, 100.0);
        assert_eq!(w.t, vec![0.0, 1.0, 2.0]);
    }

    #[test]
    fn window_slices_run_parallel() {
        let ts: Vec<f32> = (0..100).map(|i| i as f32 * 0.1).collect();
        let mut h = hist_with_times(&ts);
        h.pos_err = h.t.iter().map(|t| t * 10.0).collect();
        let w = window(&h, 1.0);
        assert_eq!(w.t.len(), w.pos_err.len(), "slices must stay aligned");
        assert!(
            (w.t.first().copied().unwrap_or(-1.0) - 8.9).abs() < 1e-3,
            "starts at 10s-1s"
        );
        assert!(
            (w.pos_err.last().copied().unwrap_or(-1.0) - 99.0).abs() < 1e-2,
            "values aligned with times"
        );
    }
}
