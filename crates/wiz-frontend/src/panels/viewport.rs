use crate::panels::Panel;
use crate::viewport_state::SharedViewportState;
use glam::Vec3;

pub struct ViewportPanel {
    last_size: egui::Vec2,
    camera_mode: CameraMode,
    #[allow(dead_code)]
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

    fn render_axes_indicator(&self, ui: &mut egui::Ui, rect: egui::Rect, yaw: f32, pitch: f32) {
        let painter = ui.painter();
        let axes_center = rect.right_bottom() - egui::vec2(50.0, 50.0);
        let axis_len = 30.0;

        // Calculate camera basis vectors from yaw and pitch
        // Camera looks from position towards target (origin)
        // yaw: rotation around Z axis, pitch: elevation angle
        let cos_yaw = yaw.cos();
        let sin_yaw = yaw.sin();
        let cos_pitch = pitch.cos();
        let sin_pitch = pitch.sin();

        // Camera's forward direction (from camera to target, normalized)
        // In orbit camera: position = target + distance * (cos_pitch*cos_yaw, cos_pitch*sin_yaw, sin_pitch)
        // Forward = -position direction = (-cos_pitch*cos_yaw, -cos_pitch*sin_yaw, -sin_pitch)
        let forward = Vec3::new(-cos_pitch * cos_yaw, -cos_pitch * sin_yaw, -sin_pitch);

        // Camera's right direction (perpendicular to forward and world up)
        let world_up = Vec3::Z;
        let right = forward.cross(world_up).normalize();

        // Camera's up direction
        let up = right.cross(forward).normalize();

        // Project world axes onto camera's screen plane (right, up)
        let project_axis = |world_axis: Vec3| -> (egui::Vec2, f32) {
            let x = world_axis.dot(right);
            let y = world_axis.dot(up);
            let z = world_axis.dot(forward); // depth for sorting
            (egui::vec2(x * axis_len, -y * axis_len), z)
        };

        let (x_dir, x_depth) = project_axis(Vec3::X);
        let (y_dir, y_depth) = project_axis(Vec3::Y);
        let (z_dir, z_depth) = project_axis(Vec3::Z);

        // Draw axes in depth order (furthest first, closest last on top)
        let mut axes = [
            (x_depth, x_dir, "X", egui::Color32::from_rgb(255, 68, 68)),
            (y_depth, y_dir, "Y", egui::Color32::from_rgb(68, 255, 68)),
            (z_depth, z_dir, "Z", egui::Color32::from_rgb(68, 68, 255)),
        ];
        // Sort by depth descending - larger depth (towards camera) drawn first
        // Smaller depth (away from camera) drawn last (on top)
        axes.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        for (_depth, dir, label, color) in axes {
            painter.line_segment(
                [axes_center, axes_center + dir],
                egui::Stroke::new(2.0, color),
            );

            // Position label at the end of the axis
            let label_offset = dir.normalized() * 8.0;
            painter.text(
                axes_center + dir + label_offset,
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::default(),
                color,
            );
        }
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

        // Get camera orientation for axes indicator before dropping state
        let yaw = state.renderer.camera.yaw;
        let pitch = state.renderer.camera.pitch;
        drop(state);

        // Draw axes indicator overlay
        self.render_axes_indicator(ui, response.rect, yaw, pitch);

        self.last_size = available_size;
    }
}
