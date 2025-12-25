use glam::Mat4;
use std::collections::HashSet;
use wgpu::util::DeviceExt;

use crate::{
    camera::Camera,
    grid::GridRenderer,
    laser_scan::{LaserScanData, LaserScanRenderer, LaserScanVertex},
    point_cloud::{PointCloudData, PointCloudRenderer, PointVertex},
    pose::{PoseInstance, PoseRenderer},
    tf_axis::TfAxisRenderer,
};

pub struct Renderer {
    pub camera: Camera,
    camera_buffer: wgpu::Buffer,
    camera_bind_group_layout: wgpu::BindGroupLayout,
    depth_texture: wgpu::Texture,
    depth_view: wgpu::TextureView,
    grid_renderer: GridRenderer,
    point_cloud_renderer: PointCloudRenderer,
    laser_scan_renderer: LaserScanRenderer,
    tf_axis_renderer: TfAxisRenderer,
    pose_renderer: PoseRenderer,
    point_clouds: Vec<PointCloudData>,
    laser_scans: Vec<LaserScanData>,
    poses: Vec<PoseInstance>,
    show_tf_frames: bool,
    show_poses: bool,
    format: wgpu::TextureFormat,
    width: u32,
    height: u32,
    /// Visible point cloud indices
    visible_point_clouds: HashSet<usize>,
    /// Visible laser scan indices
    visible_laser_scans: HashSet<usize>,
}

impl Renderer {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        width: u32,
        height: u32,
    ) -> Self {
        let depth_format = wgpu::TextureFormat::Depth32Float;

        let camera = Camera::new(width as f32 / height as f32);
        let camera_uniform = camera.uniform();

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let camera_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Camera Bind Group Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let (depth_texture, depth_view) = Self::create_depth_texture(device, width, height);

        let grid_renderer = GridRenderer::new(
            device,
            format,
            depth_format,
            &camera_bind_group_layout,
            &camera_buffer,
        );

        let point_cloud_renderer = PointCloudRenderer::new(
            device,
            format,
            depth_format,
            &camera_bind_group_layout,
            &camera_buffer,
        );

        let laser_scan_renderer = LaserScanRenderer::new(
            device,
            format,
            depth_format,
            &camera_bind_group_layout,
            &camera_buffer,
        );

        let tf_axis_renderer = TfAxisRenderer::new(
            device,
            format,
            depth_format,
            &camera_bind_group_layout,
            &camera_buffer,
        );

        let pose_renderer = PoseRenderer::new(
            device,
            format,
            depth_format,
            &camera_bind_group_layout,
            &camera_buffer,
        );

        Self {
            camera,
            camera_buffer,
            camera_bind_group_layout,
            depth_texture,
            depth_view,
            grid_renderer,
            point_cloud_renderer,
            laser_scan_renderer,
            tf_axis_renderer,
            pose_renderer,
            point_clouds: Vec::new(),
            laser_scans: Vec::new(),
            poses: Vec::new(),
            show_tf_frames: true,
            show_poses: true,
            format,
            width,
            height,
            visible_point_clouds: HashSet::new(),
            visible_laser_scans: HashSet::new(),
        }
    }

    /// Set point cloud visibility by index
    pub fn set_point_cloud_visible(&mut self, index: usize, visible: bool) {
        if visible {
            self.visible_point_clouds.insert(index);
        } else {
            self.visible_point_clouds.remove(&index);
        }
    }

    /// Set laser scan visibility by index
    pub fn set_laser_scan_visible(&mut self, index: usize, visible: bool) {
        if visible {
            self.visible_laser_scans.insert(index);
        } else {
            self.visible_laser_scans.remove(&index);
        }
    }

    fn create_depth_texture(
        device: &wgpu::Device,
        width: u32,
        height: u32,
    ) -> (wgpu::Texture, wgpu::TextureView) {
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Depth Texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        (texture, view)
    }

    pub fn resize(&mut self, device: &wgpu::Device, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.width = width;
        self.height = height;
        self.camera.update_aspect(width as f32 / height as f32);
        let (depth_texture, depth_view) = Self::create_depth_texture(device, width, height);
        self.depth_texture = depth_texture;
        self.depth_view = depth_view;
    }

    pub fn update_camera(&self, queue: &wgpu::Queue) {
        let camera_uniform = self.camera.uniform();
        queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[camera_uniform]),
        );
    }

    pub fn add_point_cloud(&mut self, device: &wgpu::Device, vertices: &[PointVertex]) -> usize {
        let data = PointCloudData::new(device, vertices);
        self.point_clouds.push(data);
        self.point_clouds.len() - 1
    }

    pub fn update_point_cloud(&self, queue: &wgpu::Queue, index: usize, vertices: &[PointVertex]) {
        if let Some(pc) = self.point_clouds.get(index) {
            pc.update(queue, vertices);
        }
    }

    pub fn clear_point_clouds(&mut self) {
        self.point_clouds.clear();
    }

    /// Add a laser scan to be rendered
    pub fn add_laser_scan(&mut self, device: &wgpu::Device, vertices: &[LaserScanVertex]) -> usize {
        let data = LaserScanData::new(device, vertices);
        self.laser_scans.push(data);
        self.laser_scans.len() - 1
    }

    /// Add a laser scan from raw scan parameters
    #[allow(clippy::too_many_arguments)]
    pub fn add_laser_scan_from_data(
        &mut self,
        device: &wgpu::Device,
        ranges: &[f32],
        angle_min: f32,
        angle_increment: f32,
        range_min: f32,
        range_max: f32,
        intensities: Option<&[f32]>,
    ) -> usize {
        let data = LaserScanData::from_scan(
            device,
            ranges,
            angle_min,
            angle_increment,
            range_min,
            range_max,
            intensities,
        );
        self.laser_scans.push(data);
        self.laser_scans.len() - 1
    }

    pub fn update_laser_scan(
        &self,
        queue: &wgpu::Queue,
        index: usize,
        vertices: &[LaserScanVertex],
    ) {
        if let Some(ls) = self.laser_scans.get(index) {
            ls.update(queue, vertices);
        }
    }

    pub fn clear_laser_scans(&mut self) {
        self.laser_scans.clear();
    }

    /// Set point cloud point size
    pub fn set_point_size(&mut self, size: f32) {
        self.point_cloud_renderer.set_point_size(size);
    }

    /// Set point cloud alpha
    pub fn set_point_alpha(&mut self, alpha: f32) {
        self.point_cloud_renderer.set_alpha(alpha);
    }

    /// Set laser scan color
    pub fn set_laser_scan_color(&mut self, r: f32, g: f32, b: f32, a: f32) {
        self.laser_scan_renderer.set_color(r, g, b, a);
    }

    /// Toggle laser scan point display mode
    pub fn set_laser_scan_show_points(&mut self, show: bool) {
        self.laser_scan_renderer.set_show_points(show);
    }

    /// Update TF frame transforms
    pub fn update_tf_frames(&mut self, queue: &wgpu::Queue, transforms: &[Mat4]) {
        self.tf_axis_renderer.update_frames(queue, transforms);
    }

    /// Set TF axis length
    pub fn set_tf_axis_length(&mut self, length: f32) {
        self.tf_axis_renderer.set_axis_length(length);
    }

    /// Toggle TF frame visibility
    pub fn set_show_tf_frames(&mut self, show: bool) {
        self.show_tf_frames = show;
    }

    /// Get TF frame visibility
    pub fn show_tf_frames(&self) -> bool {
        self.show_tf_frames
    }

    /// Update pose instances
    pub fn update_poses(&mut self, queue: &wgpu::Queue, poses: &[PoseInstance]) {
        self.poses = poses.to_vec();
        self.pose_renderer.update_poses(queue, poses);
    }

    /// Add a single pose
    pub fn add_pose(&mut self, queue: &wgpu::Queue, pose: PoseInstance) {
        self.poses.push(pose);
        self.pose_renderer.update_poses(queue, &self.poses);
    }

    /// Clear all poses
    pub fn clear_poses(&mut self, queue: &wgpu::Queue) {
        self.poses.clear();
        self.pose_renderer.update_poses(queue, &self.poses);
    }

    /// Set pose arrow length
    pub fn set_pose_arrow_length(&mut self, length: f32) {
        self.pose_renderer.set_arrow_length(length);
    }

    /// Set pose arrow width (scale factor)
    pub fn set_pose_arrow_width(&mut self, width: f32) {
        self.pose_renderer.set_arrow_width(width);
    }

    /// Set pose arrow color
    pub fn set_pose_color(&mut self, r: f32, g: f32, b: f32, a: f32) {
        self.pose_renderer.set_color(r, g, b, a);
    }

    /// Toggle pose visibility
    pub fn set_show_poses(&mut self, show: bool) {
        self.show_poses = show;
    }

    /// Get pose visibility
    pub fn show_poses(&self) -> bool {
        self.show_poses
    }

    pub fn render(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        view: &wgpu::TextureView,
        queue: &wgpu::Queue,
    ) {
        self.update_camera(queue);
        self.point_cloud_renderer.update_settings(queue);
        self.laser_scan_renderer.update_settings(queue);
        self.tf_axis_renderer.update_settings(queue);
        self.pose_renderer.update_settings(queue);

        let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("Main Render Pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.1,
                        g: 0.1,
                        b: 0.1,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        // Render grid
        self.grid_renderer.render(&mut render_pass);

        // Render visible point clouds only
        for (i, pc) in self.point_clouds.iter().enumerate() {
            if self.visible_point_clouds.contains(&i) {
                self.point_cloud_renderer.render(
                    &mut render_pass,
                    &pc.vertex_buffer,
                    pc.vertex_count,
                );
            }
        }

        // Render visible laser scans only
        for (i, ls) in self.laser_scans.iter().enumerate() {
            if self.visible_laser_scans.contains(&i) {
                self.laser_scan_renderer.render(
                    &mut render_pass,
                    &ls.vertex_buffer,
                    ls.vertex_count,
                );
            }
        }

        // Render TF frames
        if self.show_tf_frames {
            self.tf_axis_renderer.render(&mut render_pass);
        }

        // Render poses
        if self.show_poses {
            self.pose_renderer.render(&mut render_pass);
        }
    }

    pub fn format(&self) -> wgpu::TextureFormat {
        self.format
    }

    pub fn camera_bind_group_layout(&self) -> &wgpu::BindGroupLayout {
        &self.camera_bind_group_layout
    }
}
