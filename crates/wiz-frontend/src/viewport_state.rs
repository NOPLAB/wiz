use glam::Mat4;
use parking_lot::Mutex;
use std::sync::Arc;
use wiz_core::Transform3D;
use wiz_renderer::{Renderer, laser_scan::LaserScanVertex, point_cloud::PointVertex};

/// Shared viewport rendering state
pub struct ViewportState {
    pub renderer: Renderer,
    pub device: Arc<wgpu::Device>,
    pub queue: Arc<wgpu::Queue>,
    render_texture: Option<RenderTexture>,
}

struct RenderTexture {
    #[allow(dead_code)]
    texture: wgpu::Texture,
    view: wgpu::TextureView,
    egui_texture_id: egui::TextureId,
    width: u32,
    height: u32,
}

impl ViewportState {
    pub fn new(
        device: Arc<wgpu::Device>,
        queue: Arc<wgpu::Queue>,
        format: wgpu::TextureFormat,
    ) -> Self {
        let renderer = Renderer::new(&device, format, 800, 600);
        Self {
            renderer,
            device,
            queue,
            render_texture: None,
        }
    }

    /// Ensure the render texture matches the requested size
    pub fn ensure_texture(
        &mut self,
        width: u32,
        height: u32,
        egui_renderer: &mut egui_wgpu::Renderer,
    ) -> egui::TextureId {
        let width = width.max(1);
        let height = height.max(1);

        let needs_recreate = self
            .render_texture
            .as_ref()
            .is_none_or(|t| t.width != width || t.height != height);

        if needs_recreate {
            // Free old texture if exists
            if let Some(old) = self.render_texture.take() {
                egui_renderer.free_texture(&old.egui_texture_id);
            }

            // Create new texture
            let texture = self.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("Viewport Render Texture"),
                size: wgpu::Extent3d {
                    width,
                    height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: self.renderer.format(),
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                    | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            });

            let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

            // Register with egui
            let egui_texture_id = egui_renderer.register_native_texture(
                &self.device,
                &view,
                wgpu::FilterMode::Linear,
            );

            // Resize renderer
            self.renderer.resize(&self.device, width, height);

            self.render_texture = Some(RenderTexture {
                texture,
                view,
                egui_texture_id,
                width,
                height,
            });
        }

        self.render_texture.as_ref().unwrap().egui_texture_id
    }

    /// Render the 3D scene to the texture
    pub fn render(&mut self) {
        let Some(ref rt) = self.render_texture else {
            return;
        };

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Viewport Render Encoder"),
            });

        self.renderer.render(&mut encoder, &rt.view, &self.queue);

        self.queue.submit(std::iter::once(encoder.finish()));
    }

    /// Add sample point cloud for testing
    pub fn add_sample_point_cloud(&mut self) {
        let vertices: Vec<PointVertex> = (0..1000)
            .map(|i| {
                let t = i as f32 / 1000.0;
                let angle = t * std::f32::consts::PI * 20.0;
                let radius = t * 3.0;
                let x = radius * angle.cos();
                let y = radius * angle.sin();
                let z = t * 2.0;
                PointVertex {
                    position: [x, y, z],
                    color: [t, 1.0 - t, 0.5, 1.0],
                }
            })
            .collect();

        self.renderer.add_point_cloud(&self.device, &vertices);
    }

    /// Add sample laser scan for testing
    pub fn add_sample_laser_scan(&mut self) {
        // Simulate a 360-degree laser scan with some obstacles
        let num_beams = 360;
        let angle_min = 0.0_f32;
        let angle_increment = std::f32::consts::TAU / num_beams as f32;

        let vertices: Vec<LaserScanVertex> = (0..num_beams)
            .map(|i| {
                let angle = angle_min + (i as f32) * angle_increment;

                // Simulate varying ranges with some obstacles
                let base_range = 5.0;
                let obstacle_factor = ((angle * 3.0).sin().abs() * 2.0).min(3.0);
                let range = base_range - obstacle_factor;

                let x = range * angle.cos();
                let y = range * angle.sin();
                let z = 0.0;

                LaserScanVertex {
                    position: [x, y, z],
                    intensity: 1.0,
                }
            })
            .collect();

        self.renderer.set_laser_scan_color(1.0, 0.3, 0.3, 1.0);
        self.renderer.add_laser_scan(&self.device, &vertices);
    }

    /// Update TF frames from transforms
    #[allow(dead_code)]
    pub fn update_tf_frames(&mut self, transforms: &[Transform3D]) {
        let matrices: Vec<Mat4> = transforms.iter().map(|t| t.to_mat4()).collect();
        self.renderer.update_tf_frames(&self.queue, &matrices);
    }

    /// Update TF frames from Mat4 matrices
    #[allow(dead_code)]
    pub fn update_tf_frames_mat4(&mut self, transforms: &[Mat4]) {
        self.renderer.update_tf_frames(&self.queue, transforms);
    }

    /// Set TF frame visibility
    pub fn set_show_tf_frames(&mut self, show: bool) {
        self.renderer.set_show_tf_frames(show);
    }

    /// Get TF frame visibility
    #[allow(dead_code)]
    pub fn show_tf_frames(&self) -> bool {
        self.renderer.show_tf_frames()
    }

    /// Set TF axis length
    #[allow(dead_code)]
    pub fn set_tf_axis_length(&mut self, length: f32) {
        self.renderer.set_tf_axis_length(length);
    }

    /// Add sample TF frames for testing
    pub fn add_sample_tf_frames(&mut self) {
        use glam::{Quat, Vec3};

        let frames = vec![
            // Origin frame
            Mat4::IDENTITY,
            // Frame at (1, 0, 0) rotated 45 degrees around Z
            Mat4::from_rotation_translation(
                Quat::from_rotation_z(std::f32::consts::FRAC_PI_4),
                Vec3::new(1.0, 0.0, 0.0),
            ),
            // Frame at (0, 2, 0) rotated 90 degrees around X
            Mat4::from_rotation_translation(
                Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
                Vec3::new(0.0, 2.0, 0.0),
            ),
            // Frame at (1, 1, 1)
            Mat4::from_translation(Vec3::new(1.0, 1.0, 1.0)),
        ];

        self.renderer.update_tf_frames(&self.queue, &frames);
    }
}

pub type SharedViewportState = Arc<Mutex<ViewportState>>;
