mod app;
mod app_state;
mod panels;
mod viewport_state;
mod ws_client;

pub use app::WizApp;

/// WASM entry point
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub async fn start() -> Result<(), JsValue> {
    // Set up panic hook for better error messages
    console_error_panic_hook::set_once();

    // Initialize tracing for WASM (without timestamps - not supported in WASM)
    use tracing_subscriber::layer::SubscriberExt;
    use tracing_subscriber::util::SubscriberInitExt;
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::fmt::layer()
                .without_time()
                .with_writer(tracing_web::MakeWebConsoleWriter::new()),
        )
        .try_init()
        .ok();

    tracing::info!("Starting wiz frontend (WASM)");

    // Get the canvas element from the DOM
    let document = web_sys::window()
        .expect("No window")
        .document()
        .expect("No document");

    let canvas = document
        .get_element_by_id("wiz_canvas")
        .expect("Failed to find canvas element")
        .dyn_into::<web_sys::HtmlCanvasElement>()
        .expect("Element is not a canvas");

    let web_options = eframe::WebOptions::default();

    wasm_bindgen_futures::spawn_local(async move {
        let result = eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|cc| Ok(Box::new(WizApp::new(cc)))),
            )
            .await;

        if let Err(e) = result {
            tracing::error!("Failed to start eframe: {:?}", e);
        }
    });

    Ok(())
}
