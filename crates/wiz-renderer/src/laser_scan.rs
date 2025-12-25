use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

use crate::pipeline::PipelineBuilder;

/// Vertex data for laser scan points
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct LaserScanVertex {
    pub position: [f32; 3],
    pub intensity: f32,
}

/// Settings for laser scan rendering
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct LaserScanSettings {
    color: [f32; 4],
    line_width: f32,
    show_points: u32,
    _padding: [f32; 2],
}

/// Renderer for laser scan data
pub struct LaserScanRenderer {
    line_pipeline: wgpu::RenderPipeline,
    point_pipeline: wgpu::RenderPipeline,
    settings_buffer: wgpu::Buffer,
    settings_bind_group: wgpu::BindGroup,
    camera_bind_group: wgpu::BindGroup,
    color: [f32; 4],
    line_width: f32,
    show_points: bool,
}

impl LaserScanRenderer {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        depth_format: wgpu::TextureFormat,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
        camera_buffer: &wgpu::Buffer,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Laser Scan Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/laser_scan.wgsl").into()),
        });

        let settings = LaserScanSettings {
            color: [1.0, 0.0, 0.0, 1.0], // Default red
            line_width: 1.0,
            show_points: 0,
            _padding: [0.0; 2],
        };

        let settings_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Laser Scan Settings Buffer"),
            contents: bytemuck::cast_slice(&[settings]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let settings_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Laser Scan Settings Bind Group Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let settings_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Laser Scan Settings Bind Group"),
            layout: &settings_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: settings_buffer.as_entire_binding(),
            }],
        });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Laser Scan Camera Bind Group"),
            layout: camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<LaserScanVertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: 12,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32,
                },
            ],
        };

        // Pipeline for line strip rendering
        let line_pipeline = PipelineBuilder::new(device)
            .shader(&shader)
            .vertex_layout(vertex_layout.clone())
            .bind_group_layout(camera_bind_group_layout)
            .bind_group_layout(&settings_bind_group_layout)
            .format(format)
            .depth_format(depth_format)
            .topology(wgpu::PrimitiveTopology::LineStrip)
            .label("Laser Scan Line Pipeline")
            .build();

        // Pipeline for point rendering
        let point_pipeline = PipelineBuilder::new(device)
            .shader(&shader)
            .vertex_layout(vertex_layout)
            .bind_group_layout(camera_bind_group_layout)
            .bind_group_layout(&settings_bind_group_layout)
            .format(format)
            .depth_format(depth_format)
            .topology(wgpu::PrimitiveTopology::PointList)
            .label("Laser Scan Point Pipeline")
            .build();

        Self {
            line_pipeline,
            point_pipeline,
            settings_buffer,
            settings_bind_group,
            camera_bind_group,
            color: [1.0, 0.0, 0.0, 1.0],
            line_width: 1.0,
            show_points: false,
        }
    }

    pub fn set_color(&mut self, r: f32, g: f32, b: f32, a: f32) {
        self.color = [r, g, b, a];
    }

    pub fn set_show_points(&mut self, show: bool) {
        self.show_points = show;
    }

    pub fn update_settings(&self, queue: &wgpu::Queue) {
        let settings = LaserScanSettings {
            color: self.color,
            line_width: self.line_width,
            show_points: if self.show_points { 1 } else { 0 },
            _padding: [0.0; 2],
        };
        queue.write_buffer(&self.settings_buffer, 0, bytemuck::cast_slice(&[settings]));
    }

    pub fn render<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        vertex_buffer: &'a wgpu::Buffer,
        vertex_count: u32,
    ) {
        // Choose pipeline based on display mode
        if self.show_points {
            render_pass.set_pipeline(&self.point_pipeline);
        } else {
            render_pass.set_pipeline(&self.line_pipeline);
        }

        render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
        render_pass.set_bind_group(1, &self.settings_bind_group, &[]);
        render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
        render_pass.draw(0..vertex_count, 0..1);
    }
}

/// Data for a single laser scan
pub struct LaserScanData {
    pub vertex_buffer: wgpu::Buffer,
    pub vertex_count: u32,
}

impl LaserScanData {
    /// Create laser scan data from raw scan parameters
    pub fn from_scan(
        device: &wgpu::Device,
        ranges: &[f32],
        angle_min: f32,
        angle_increment: f32,
        range_min: f32,
        range_max: f32,
        intensities: Option<&[f32]>,
    ) -> Self {
        let vertices: Vec<LaserScanVertex> = ranges
            .iter()
            .enumerate()
            .filter_map(|(i, &range)| {
                // Filter out invalid ranges
                if range < range_min || range > range_max || !range.is_finite() {
                    return None;
                }

                let angle = angle_min + (i as f32) * angle_increment;
                let x = range * angle.cos();
                let y = range * angle.sin();
                let z = 0.0; // 2D scan is in XY plane

                let intensity = intensities
                    .and_then(|ints| ints.get(i).copied())
                    .unwrap_or(1.0);

                Some(LaserScanVertex {
                    position: [x, y, z],
                    intensity,
                })
            })
            .collect();

        Self::new(device, &vertices)
    }

    /// Create laser scan data from pre-computed vertices
    pub fn new(device: &wgpu::Device, vertices: &[LaserScanVertex]) -> Self {
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Laser Scan Vertex Buffer"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });

        Self {
            vertex_buffer,
            vertex_count: vertices.len() as u32,
        }
    }

    pub fn update(&self, queue: &wgpu::Queue, vertices: &[LaserScanVertex]) {
        queue.write_buffer(&self.vertex_buffer, 0, bytemuck::cast_slice(vertices));
    }
}
