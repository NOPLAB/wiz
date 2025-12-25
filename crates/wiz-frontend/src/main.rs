mod app;
mod panels;
mod viewport_state;
mod ws_client;

use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

fn main() -> eframe::Result<()> {
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "wiz_frontend=debug".into()),
        )
        .with(tracing_subscriber::fmt::layer())
        .init();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 720.0])
            .with_min_inner_size([800.0, 600.0])
            .with_title("wiz - ROS2 Visualization"),
        ..Default::default()
    };

    eframe::run_native(
        "wiz",
        native_options,
        Box::new(|cc| Ok(Box::new(app::WizApp::new(cc)))),
    )
}
