//! 2D 시각화 (Task 10에서 구현).
use crate::sim::Sim;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum View {
    Side,
    Top,
}

pub fn draw_central(ui: &mut egui::Ui, sim: &Sim, _view: View) {
    ui.heading("시각화 (Task 10에서 구현)");
    ui.label(format!("t = {:.1} s", sim.t));
}
