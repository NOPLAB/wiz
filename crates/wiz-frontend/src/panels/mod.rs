mod displays;
mod performance;
mod tf_tree;
mod topics;
mod viewport;

pub use displays::DisplaysPanel;
pub use performance::PerformancePanel;
pub use tf_tree::TfTreePanel;
pub use topics::TopicsPanel;
pub use viewport::ViewportPanel;

use crate::viewport_state::SharedViewportState;

pub trait Panel {
    fn name(&self) -> &str;
    fn ui(&mut self, ui: &mut egui::Ui);

    /// UI with render context for panels that need wgpu access
    fn ui_with_render_context(
        &mut self,
        ui: &mut egui::Ui,
        _render_state: &egui_wgpu::RenderState,
        _viewport_state: SharedViewportState,
    ) {
        // Default implementation just calls ui()
        self.ui(ui);
    }
}
