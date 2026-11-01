//! 실시간 그래프 (Task 11에서 구현).
use crate::sim::Sim;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlotTab {
    PositionError,
    Attitude,
    MotorOutputs,
}

pub fn draw_bottom(ui: &mut egui::Ui, _sim: &Sim, tab: &mut PlotTab) {
    ui.horizontal(|ui| {
        ui.selectable_value(tab, PlotTab::PositionError, "위치 오차");
        ui.selectable_value(tab, PlotTab::Attitude, "자세");
        ui.selectable_value(tab, PlotTab::MotorOutputs, "모터 출력");
    });
    ui.label("그래프 (Task 11에서 구현)");
}
