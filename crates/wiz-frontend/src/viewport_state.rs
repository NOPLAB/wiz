use glam::Mat4;
use parking_lot::Mutex;
use std::collections::HashMap;
use std::sync::Arc;
use wiz_core::{LaserScan, PointCloud2, PoseStamped, Transform3D};
use wiz_renderer::{PoseInstance, Renderer, laser_scan::LaserScanVertex, point_cloud::PointVertex};

/// Shared viewport rendering state
pub struct ViewportState {
    pub renderer: Renderer,
    pub device: Arc<wgpu::Device>,
    pub queue: Arc<wgpu::Queue>,
    render_texture: Option<RenderTexture>,
    /// Map from topic name to point cloud index in renderer
    point_cloud_indices: HashMap<String, usize>,
    /// Map from topic name to laser scan index in renderer
    laser_scan_indices: HashMap<String, usize>,
    /// Map from topic name to pose data
    pose_data: HashMap<String, PoseInstance>,
    /// Topic visibility (true = visible in 3D view)
    topic_visibility: HashMap<String, bool>,
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
            point_cloud_indices: HashMap::new(),
            laser_scan_indices: HashMap::new(),
            pose_data: HashMap::new(),
            topic_visibility: HashMap::new(),
        }
    }

    /// Set topic visibility for 3D rendering
    pub fn set_topic_visibility(&mut self, topic: &str, visible: bool) {
        self.topic_visibility.insert(topic.to_string(), visible);

        // Update renderer visibility
        if let Some(&index) = self.point_cloud_indices.get(topic) {
            self.renderer.set_point_cloud_visible(index, visible);
        }
        if let Some(&index) = self.laser_scan_indices.get(topic) {
            self.renderer.set_laser_scan_visible(index, visible);
        }
    }

    /// Check if a topic is visible
    pub fn is_topic_visible(&self, topic: &str) -> bool {
        *self.topic_visibility.get(topic).unwrap_or(&true)
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
    #[allow(dead_code)]
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
    #[allow(dead_code)]
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

    /// Set point cloud point size
    pub fn set_point_size(&mut self, size: f32) {
        self.renderer.set_point_size(size);
    }

    /// Set point cloud alpha
    pub fn set_point_alpha(&mut self, alpha: f32) {
        self.renderer.set_point_alpha(alpha);
    }

    /// Set laser scan color
    pub fn set_laser_scan_color(&mut self, color: [f32; 4]) {
        self.renderer
            .set_laser_scan_color(color[0], color[1], color[2], color[3]);
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
    #[allow(dead_code)]
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

    /// Update point cloud from PointCloud2 message for a specific topic
    pub fn update_point_cloud(&mut self, topic: &str, cloud: &PointCloud2) {
        let vertices = self.point_cloud_to_vertices(cloud);
        if vertices.is_empty() {
            return;
        }

        if let Some(&index) = self.point_cloud_indices.get(topic) {
            // Update existing point cloud
            self.renderer
                .update_point_cloud(&self.queue, index, &vertices);
        } else {
            // Add new point cloud and track the index
            let index = self.renderer.add_point_cloud(&self.device, &vertices);
            self.point_cloud_indices.insert(topic.to_string(), index);

            // Set visibility based on topic_visibility (default to visible)
            let visible = self.is_topic_visible(topic);
            self.renderer.set_point_cloud_visible(index, visible);
        }
    }

    /// Update laser scan from LaserScan message for a specific topic
    pub fn update_laser_scan(&mut self, topic: &str, scan: &LaserScan) {
        let vertices = self.laser_scan_to_vertices(scan);
        if vertices.is_empty() {
            return;
        }

        if let Some(&index) = self.laser_scan_indices.get(topic) {
            // Update existing laser scan
            self.renderer
                .update_laser_scan(&self.queue, index, &vertices);
        } else {
            // Add new laser scan and track the index
            let index = self.renderer.add_laser_scan(&self.device, &vertices);
            self.laser_scan_indices.insert(topic.to_string(), index);

            // Set visibility based on topic_visibility (default to visible)
            let visible = self.is_topic_visible(topic);
            self.renderer.set_laser_scan_visible(index, visible);
        }
    }

    /// Update pose from PoseStamped message for a specific topic
    pub fn update_pose(&mut self, topic: &str, pose: &PoseStamped) {
        let instance = PoseInstance::from_pose(pose.pose.position, pose.pose.orientation);
        self.pose_data.insert(topic.to_string(), instance);

        // Update renderer with all poses
        let poses: Vec<PoseInstance> = self.pose_data.values().cloned().collect();
        self.renderer.update_poses(&self.queue, &poses);
    }

    /// Set pose arrow length
    pub fn set_pose_arrow_length(&mut self, length: f32) {
        self.renderer.set_pose_arrow_length(length);
    }

    /// Set pose arrow width
    pub fn set_pose_arrow_width(&mut self, width: f32) {
        self.renderer.set_pose_arrow_width(width);
    }

    /// Set pose color
    pub fn set_pose_color(&mut self, color: [f32; 4]) {
        self.renderer
            .set_pose_color(color[0], color[1], color[2], color[3]);
    }

    /// Toggle pose visibility
    #[allow(dead_code)]
    pub fn set_show_poses(&mut self, show: bool) {
        self.renderer.set_show_poses(show);
    }

    /// Get pose visibility
    #[allow(dead_code)]
    pub fn show_poses(&self) -> bool {
        self.renderer.show_poses()
    }

    /// Convert PointCloud2 to PointVertex array
    fn point_cloud_to_vertices(&self, cloud: &PointCloud2) -> Vec<PointVertex> {
        let x_field = cloud.find_field("x");
        let y_field = cloud.find_field("y");
        let z_field = cloud.find_field("z");
        let rgba_field = cloud.find_field("rgba");
        let rgb_field = cloud.find_field("rgb");

        let (Some(x_field), Some(y_field), Some(z_field)) = (x_field, y_field, z_field) else {
            tracing::warn!("PointCloud2 missing x/y/z fields");
            return Vec::new();
        };

        let point_count = cloud.point_count();
        let mut vertices = Vec::with_capacity(point_count);

        for i in 0..point_count {
            let offset = i * cloud.point_step as usize;

            let x = read_f32(&cloud.data, offset + x_field.offset as usize);
            let y = read_f32(&cloud.data, offset + y_field.offset as usize);
            let z = read_f32(&cloud.data, offset + z_field.offset as usize);

            let color = if let Some(rgba) = rgba_field {
                let rgba_val = read_u32(&cloud.data, offset + rgba.offset as usize);
                [
                    (rgba_val & 0xFF) as f32 / 255.0,         // R
                    ((rgba_val >> 8) & 0xFF) as f32 / 255.0,  // G
                    ((rgba_val >> 16) & 0xFF) as f32 / 255.0, // B
                    ((rgba_val >> 24) & 0xFF) as f32 / 255.0, // A
                ]
            } else if let Some(rgb) = rgb_field {
                let rgb_val = read_u32(&cloud.data, offset + rgb.offset as usize);
                [
                    ((rgb_val >> 16) & 0xFF) as f32 / 255.0, // R
                    ((rgb_val >> 8) & 0xFF) as f32 / 255.0,  // G
                    (rgb_val & 0xFF) as f32 / 255.0,         // B
                    1.0,                                     // A
                ]
            } else {
                // Default color based on height
                let t = (z / 3.0).clamp(0.0, 1.0);
                [t, 0.5, 1.0 - t, 1.0]
            };

            vertices.push(PointVertex {
                position: [x, y, z],
                color,
            });
        }

        vertices
    }

    /// Convert LaserScan to LaserScanVertex array
    fn laser_scan_to_vertices(&self, scan: &LaserScan) -> Vec<LaserScanVertex> {
        let mut vertices = Vec::with_capacity(scan.ranges.len());
        let mut angle = scan.angle_min;

        for (i, &range) in scan.ranges.iter().enumerate() {
            if range >= scan.range_min && range <= scan.range_max && range.is_finite() {
                let x = range * angle.cos();
                let y = range * angle.sin();
                let intensity = if i < scan.intensities.len() {
                    scan.intensities[i]
                } else {
                    1.0
                };

                vertices.push(LaserScanVertex {
                    position: [x, y, 0.0],
                    intensity,
                });
            }
            angle += scan.angle_increment;
        }

        vertices
    }

    /// Clear all point clouds, laser scans, and poses
    #[allow(dead_code)]
    pub fn clear_data(&mut self) {
        self.renderer.clear_point_clouds();
        self.renderer.clear_laser_scans();
        self.renderer.clear_poses(&self.queue);
        self.point_cloud_indices.clear();
        self.laser_scan_indices.clear();
        self.pose_data.clear();
    }

    /// Remove a specific topic's data
    #[allow(dead_code)]
    pub fn remove_topic(&mut self, topic: &str) {
        // Note: The renderer doesn't support removing individual items,
        // so we just remove the tracking. The data will be overwritten
        // when a new subscription starts.
        self.point_cloud_indices.remove(topic);
        self.laser_scan_indices.remove(topic);
        if self.pose_data.remove(topic).is_some() {
            // Update renderer with remaining poses
            let poses: Vec<PoseInstance> = self.pose_data.values().cloned().collect();
            self.renderer.update_poses(&self.queue, &poses);
        }
    }
}

/// Helper to read f32 from byte slice
fn read_f32(data: &[u8], offset: usize) -> f32 {
    if offset + 4 <= data.len() {
        f32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ])
    } else {
        0.0
    }
}

/// Helper to read u32 from byte slice
fn read_u32(data: &[u8], offset: usize) -> u32 {
    if offset + 4 <= data.len() {
        u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ])
    } else {
        0
    }
}

pub type SharedViewportState = Arc<Mutex<ViewportState>>;
