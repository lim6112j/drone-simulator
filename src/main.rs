#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use eframe::egui::{self, FontData, FontDefinitions};
use drone_training::app::DroneApp;

/// 시스템 한글 폰트를 egui에 등록 (플랫폼별 후보 경로 순회).
fn install_korean_font(ctx: &egui::Context) {
    let candidates = [
        "/System/Library/Fonts/AppleSDGothicNeo.ttc",      // macOS
        "/System/Library/Fonts/Supplemental/Apple SD Gothic Neo.ttc",
        "/System/Library/Fonts/AppleGothic.ttf",           // macOS 구버전
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", // Linux
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
        "C:\\Windows\\Fonts\\malgun.ttf",                 // Windows
    ];
    for path in candidates {
        if let Ok(bytes) = std::fs::read(path) {
            let mut fonts = FontDefinitions::default();
            fonts
                .font_data
                .insert("korean".to_owned(), FontData::from_owned(bytes).into());
            for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
                if let Some(list) = fonts.families.get_mut(&family) {
                    list.push("korean".to_owned()); // 기본 글꼴 뒤 폴백으로 추가
                }
            }
            ctx.set_fonts(fonts);
            return;
        }
    }
    eprintln!("한글 폰트를 찾지 못했습니다 — UI 텍스트가 깨질 수 있습니다.");
}

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
        Box::new(|cc| {
            install_korean_font(&cc.egui_ctx);
            Ok(Box::new(DroneApp::new()))
        }),
    )
}
