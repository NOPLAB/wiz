use crate::panels::Panel;

pub struct ViewportPanel {
    last_size: egui::Vec2,
    camera_mode: CameraMode,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum CameraMode {
    Perspective,
    Orthographic,
}

impl ViewportPanel {
    pub fn new() -> Self {
        Self {
            last_size: egui::Vec2::ZERO,
            camera_mode: CameraMode::Perspective,
        }
    }
}

impl Panel for ViewportPanel {
    fn name(&self) -> &str {
        "3D Viewport"
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        // Overlay UI at top
        ui.horizontal(|ui| {
            ui.label("Frame: map");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("\u{2699}").clicked() {
                    // Settings
                }
            });
        });

        // Main viewport area
        let available_size = ui.available_size();
        let (response, painter) = ui.allocate_painter(available_size, egui::Sense::click_and_drag());

        // Draw background
        painter.rect_filled(response.rect, 0.0, egui::Color32::from_rgb(30, 30, 30));

        // Draw coordinate axes indicator in corner
        let axes_center = response.rect.right_bottom() - egui::vec2(50.0, 50.0);
        let axis_len = 30.0;

        // X axis (red)
        painter.line_segment(
            [axes_center, axes_center + egui::vec2(axis_len, 0.0)],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 68, 68)),
        );
        painter.text(
            axes_center + egui::vec2(axis_len + 5.0, 0.0),
            egui::Align2::LEFT_CENTER,
            "X",
            egui::FontId::default(),
            egui::Color32::from_rgb(255, 68, 68),
        );

        // Y axis (green)
        painter.line_segment(
            [axes_center, axes_center + egui::vec2(0.0, -axis_len)],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(68, 255, 68)),
        );
        painter.text(
            axes_center + egui::vec2(0.0, -axis_len - 5.0),
            egui::Align2::CENTER_BOTTOM,
            "Y",
            egui::FontId::default(),
            egui::Color32::from_rgb(68, 255, 68),
        );

        // Z axis (blue) - pointing "out"
        painter.line_segment(
            [axes_center, axes_center + egui::vec2(-axis_len * 0.7, -axis_len * 0.7)],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(68, 68, 255)),
        );
        painter.text(
            axes_center + egui::vec2(-axis_len * 0.7 - 5.0, -axis_len * 0.7 - 5.0),
            egui::Align2::RIGHT_BOTTOM,
            "Z",
            egui::FontId::default(),
            egui::Color32::from_rgb(68, 68, 255),
        );

        // Camera mode indicator
        let mode_text = match self.camera_mode {
            CameraMode::Perspective => "Persp",
            CameraMode::Orthographic => "Ortho",
        };
        let mode_pos = response.rect.right_bottom() - egui::vec2(10.0, 10.0);
        if ui
            .put(
                egui::Rect::from_min_size(mode_pos - egui::vec2(40.0, 20.0), egui::vec2(40.0, 20.0)),
                egui::Button::new(mode_text),
            )
            .clicked()
        {
            self.camera_mode = match self.camera_mode {
                CameraMode::Perspective => CameraMode::Orthographic,
                CameraMode::Orthographic => CameraMode::Perspective,
            };
        }

        // Handle camera input
        if response.dragged_by(egui::PointerButton::Middle) {
            let _delta = response.drag_delta();
            if ui.input(|i| i.modifiers.shift) {
                // Pan
                // TODO: Update camera pan
            } else {
                // Orbit
                // TODO: Update camera orbit
            }
        }

        // Zoom with scroll
        let scroll_delta = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll_delta != 0.0 && response.hovered() {
            // TODO: Update camera zoom
        }

        // Placeholder text
        painter.text(
            response.rect.center(),
            egui::Align2::CENTER_CENTER,
            "3D Viewport\n(WebGPU rendering will be integrated here)",
            egui::FontId::proportional(16.0),
            egui::Color32::GRAY,
        );

        self.last_size = available_size;
    }
}
