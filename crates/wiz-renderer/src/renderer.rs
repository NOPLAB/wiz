use wgpu::util::DeviceExt;

use crate::{
    camera::Camera,
    grid::GridRenderer,
    laser_scan::{LaserScanData, LaserScanRenderer, LaserScanVertex},
    point_cloud::{PointCloudData, PointCloudRenderer, PointVertex},
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
    point_clouds: Vec<PointCloudData>,
    laser_scans: Vec<LaserScanData>,
    format: wgpu::TextureFormat,
    width: u32,
    height: u32,
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

        Self {
            camera,
            camera_buffer,
            camera_bind_group_layout,
            depth_texture,
            depth_view,
            grid_renderer,
            point_cloud_renderer,
            laser_scan_renderer,
            point_clouds: Vec::new(),
            laser_scans: Vec::new(),
            format,
            width,
            height,
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

    /// Set laser scan color
    pub fn set_laser_scan_color(&mut self, r: f32, g: f32, b: f32, a: f32) {
        self.laser_scan_renderer.set_color(r, g, b, a);
    }

    /// Toggle laser scan point display mode
    pub fn set_laser_scan_show_points(&mut self, show: bool) {
        self.laser_scan_renderer.set_show_points(show);
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

        // Render point clouds
        for pc in &self.point_clouds {
            self.point_cloud_renderer
                .render(&mut render_pass, &pc.vertex_buffer, pc.vertex_count);
        }

        // Render laser scans
        for ls in &self.laser_scans {
            self.laser_scan_renderer
                .render(&mut render_pass, &ls.vertex_buffer, ls.vertex_count);
        }
    }

    pub fn format(&self) -> wgpu::TextureFormat {
        self.format
    }

    pub fn camera_bind_group_layout(&self) -> &wgpu::BindGroupLayout {
        &self.camera_bind_group_layout
    }
}
