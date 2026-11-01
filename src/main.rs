#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use drone_training::app::DroneApp;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("드론 제어 시뮬레이터"),
        ..Default::default()
    };
    eframe::run_native(
        "드론 제어 시뮬레이터",
        options,
        Box::new(|_cc| Ok(Box::new(DroneApp::new()))),
    )
}
