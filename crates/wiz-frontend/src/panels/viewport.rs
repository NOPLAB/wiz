use crate::panels::Panel;
use crate::viewport_state::SharedViewportState;

pub struct ViewportPanel {
    last_size: egui::Vec2,
    camera_mode: CameraMode,
    viewport_state: Option<SharedViewportState>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum CameraMode {
    Perspective,
    Orthographic,
}

impl ViewportPanel {
    pub fn new(viewport_state: Option<SharedViewportState>) -> Self {
        Self {
            last_size: egui::Vec2::ZERO,
            camera_mode: CameraMode::Perspective,
            viewport_state,
        }
    }

    fn render_axes_indicator(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        let painter = ui.painter();
        let axes_center = rect.right_bottom() - egui::vec2(50.0, 50.0);
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

        // Z axis (blue)
        painter.line_segment(
            [
                axes_center,
                axes_center + egui::vec2(-axis_len * 0.7, -axis_len * 0.7),
            ],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(68, 68, 255)),
        );
        painter.text(
            axes_center + egui::vec2(-axis_len * 0.7 - 5.0, -axis_len * 0.7 - 5.0),
            egui::Align2::RIGHT_BOTTOM,
            "Z",
            egui::FontId::default(),
            egui::Color32::from_rgb(68, 68, 255),
        );
    }
}

impl Panel for ViewportPanel {
    fn name(&self) -> &str {
        "3D Viewport"
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        // Fallback when no render context available
        let available_size = ui.available_size();
        let (response, painter) =
            ui.allocate_painter(available_size, egui::Sense::click_and_drag());

        painter.rect_filled(response.rect, 0.0, egui::Color32::from_rgb(30, 30, 30));
        painter.text(
            response.rect.center(),
            egui::Align2::CENTER_CENTER,
            "3D Viewport\n(WebGPU not available)",
            egui::FontId::proportional(16.0),
            egui::Color32::GRAY,
        );

        self.last_size = available_size;
    }

    fn ui_with_render_context(
        &mut self,
        ui: &mut egui::Ui,
        render_state: &egui_wgpu::RenderState,
        viewport_state: SharedViewportState,
    ) {
        // Toolbar overlay
        ui.horizontal(|ui| {
            ui.label("Frame: map");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Camera mode toggle
                let mode_text = match self.camera_mode {
                    CameraMode::Perspective => "Persp",
                    CameraMode::Orthographic => "Ortho",
                };
                if ui.button(mode_text).clicked() {
                    self.camera_mode = match self.camera_mode {
                        CameraMode::Perspective => CameraMode::Orthographic,
                        CameraMode::Orthographic => CameraMode::Perspective,
                    };
                }

                if ui.button("\u{2699}").clicked() {
                    // Settings
                }
            });
        });

        // Main viewport area
        let available_size = ui.available_size();
        let width = available_size.x as u32;
        let height = available_size.y as u32;

        if width == 0 || height == 0 {
            return;
        }

        // Ensure texture and render
        let texture_id = {
            let mut state = viewport_state.lock();
            let mut egui_renderer = render_state.renderer.write();
            let tex_id = state.ensure_texture(width, height, &mut egui_renderer);
            state.render();
            tex_id
        };

        // Display the rendered texture
        let response = ui.add(
            egui::Image::new(egui::load::SizedTexture::new(
                texture_id,
                [available_size.x, available_size.y],
            ))
            .sense(egui::Sense::click_and_drag()),
        );

        // Handle camera input
        let mut state = viewport_state.lock();

        if response.dragged_by(egui::PointerButton::Middle) {
            let delta = response.drag_delta();
            if ui.input(|i| i.modifiers.shift) {
                // Pan
                state.renderer.camera.pan(delta.x, delta.y);
            } else {
                // Orbit
                let sensitivity = 0.005;
                state
                    .renderer
                    .camera
                    .orbit(-delta.x * sensitivity, delta.y * sensitivity);
            }
        }

        // Zoom with scroll
        if response.hovered() {
            let scroll_delta = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll_delta != 0.0 {
                state.renderer.camera.zoom(scroll_delta * 0.01);
            }
        }

        // Right-click context menu
        response.context_menu(|ui| {
            if ui.button("Reset View").clicked() {
                state.renderer.camera.fit_all(glam::Vec3::ZERO, 5.0);
                ui.close_menu();
            }
            if ui.button("Top View").clicked() {
                state.renderer.camera.yaw = 0.0;
                state.renderer.camera.pitch = 89.0_f32.to_radians();
                state.renderer.camera.orbit(0.0, 0.0);
                ui.close_menu();
            }
            if ui.button("Front View").clicked() {
                state.renderer.camera.yaw = 0.0;
                state.renderer.camera.pitch = 0.0;
                state.renderer.camera.orbit(0.0, 0.0);
                ui.close_menu();
            }
            if ui.button("Side View").clicked() {
                state.renderer.camera.yaw = 90.0_f32.to_radians();
                state.renderer.camera.pitch = 0.0;
                state.renderer.camera.orbit(0.0, 0.0);
                ui.close_menu();
            }
        });

        drop(state);

        // Draw axes indicator overlay
        self.render_axes_indicator(ui, response.rect);

        self.last_size = available_size;
    }
}
