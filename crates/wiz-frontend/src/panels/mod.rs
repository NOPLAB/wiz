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

pub trait Panel {
    fn name(&self) -> &str;
    fn ui(&mut self, ui: &mut egui::Ui);
}
