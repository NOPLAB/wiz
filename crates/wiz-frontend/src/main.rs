// Native entry point
// For WASM, see lib.rs

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "wiz_frontend=debug".into()),
        )
        .with(tracing_subscriber::fmt::layer())
        .init();

    tracing::info!("Starting wiz frontend (native)");

    // Configure wgpu for better compatibility (especially WSL2 with llvmpipe)
    let wgpu_options = egui_wgpu::WgpuConfiguration {
        wgpu_setup: egui_wgpu::WgpuSetup::CreateNew {
            // Use GL backend for llvmpipe compatibility
            supported_backends: wgpu::Backends::GL,
            power_preference: wgpu::PowerPreference::LowPower,
            device_descriptor: std::sync::Arc::new(|_adapter| wgpu::DeviceDescriptor {
                label: Some("wiz device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::downlevel_webgl2_defaults(),
                memory_hints: wgpu::MemoryHints::default(),
            }),
        },
        ..Default::default()
    };

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 720.0])
            .with_min_inner_size([800.0, 600.0])
            .with_title("wiz - ROS2 Visualization"),
        wgpu_options,
        ..Default::default()
    };

    eframe::run_native(
        "wiz",
        native_options,
        Box::new(|cc| Ok(Box::new(wiz_frontend::WizApp::new(cc)))),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {
    // WASM entry point is handled by lib.rs via wasm_bindgen(start)
}
