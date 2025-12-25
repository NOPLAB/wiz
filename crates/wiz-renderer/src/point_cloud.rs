use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

use crate::pipeline::PipelineBuilder;

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct PointVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct PointCloudSettings {
    point_size: f32,
    alpha: f32,
    _padding: [f32; 2],
}

pub struct PointCloudRenderer {
    pipeline: wgpu::RenderPipeline,
    settings_buffer: wgpu::Buffer,
    settings_bind_group: wgpu::BindGroup,
    camera_bind_group: wgpu::BindGroup,
    point_size: f32,
    alpha: f32,
}

impl PointCloudRenderer {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        depth_format: wgpu::TextureFormat,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
        camera_buffer: &wgpu::Buffer,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Point Cloud Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/point_cloud.wgsl").into()),
        });

        let settings = PointCloudSettings {
            point_size: 2.0,
            alpha: 1.0,
            _padding: [0.0; 2],
        };

        let settings_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Point Cloud Settings Buffer"),
            contents: bytemuck::cast_slice(&[settings]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let settings_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Point Cloud Settings Bind Group Layout"),
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

        let settings_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Point Cloud Settings Bind Group"),
            layout: &settings_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: settings_buffer.as_entire_binding(),
            }],
        });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Point Cloud Camera Bind Group"),
            layout: camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        let pipeline = PipelineBuilder::new(device)
            .shader(&shader)
            .vertex_layout(wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<PointVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Instance, // Each point is an instance
                attributes: &[
                    wgpu::VertexAttribute {
                        offset: 0,
                        shader_location: 0,
                        format: wgpu::VertexFormat::Float32x3,
                    },
                    wgpu::VertexAttribute {
                        offset: 12,
                        shader_location: 1,
                        format: wgpu::VertexFormat::Float32x4,
                    },
                ],
            })
            .bind_group_layout(camera_bind_group_layout)
            .bind_group_layout(&settings_bind_group_layout)
            .format(format)
            .depth_format(depth_format)
            .topology(wgpu::PrimitiveTopology::TriangleList) // Draw quads as triangles
            .label("Point Cloud Pipeline")
            .build();

        Self {
            pipeline,
            settings_buffer,
            settings_bind_group,
            camera_bind_group,
            point_size: 2.0,
            alpha: 1.0,
        }
    }

    pub fn set_point_size(&mut self, size: f32) {
        self.point_size = size;
    }

    pub fn set_alpha(&mut self, alpha: f32) {
        self.alpha = alpha;
    }

    pub fn update_settings(&self, queue: &wgpu::Queue) {
        let settings = PointCloudSettings {
            point_size: self.point_size,
            alpha: self.alpha,
            _padding: [0.0; 2],
        };
        queue.write_buffer(&self.settings_buffer, 0, bytemuck::cast_slice(&[settings]));
    }

    pub fn render<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        vertex_buffer: &'a wgpu::Buffer,
        instance_count: u32,
    ) {
        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
        render_pass.set_bind_group(1, &self.settings_bind_group, &[]);
        render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
        // Draw 6 vertices (2 triangles) per instance (point)
        render_pass.draw(0..6, 0..instance_count);
    }
}

pub struct PointCloudData {
    pub vertex_buffer: wgpu::Buffer,
    pub vertex_count: u32,
}

impl PointCloudData {
    pub fn new(device: &wgpu::Device, vertices: &[PointVertex]) -> Self {
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Point Cloud Vertex Buffer"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });

        Self {
            vertex_buffer,
            vertex_count: vertices.len() as u32,
        }
    }

    pub fn update(&self, queue: &wgpu::Queue, vertices: &[PointVertex]) {
        queue.write_buffer(&self.vertex_buffer, 0, bytemuck::cast_slice(vertices));
    }
}
