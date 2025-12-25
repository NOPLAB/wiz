use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

use crate::pipeline::PipelineBuilder;

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct GridVertex {
    position: [f32; 3],
    color: [f32; 4],
}

pub struct GridRenderer {
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
    bind_group: wgpu::BindGroup,
}

impl GridRenderer {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        depth_format: wgpu::TextureFormat,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
        camera_buffer: &wgpu::Buffer,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Grid Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/grid.wgsl").into()),
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Grid Bind Group"),
            layout: camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        let pipeline = PipelineBuilder::new(device)
            .shader(&shader)
            .vertex_layout(wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<GridVertex>() as u64,
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
                        format: wgpu::VertexFormat::Float32x4,
                    },
                ],
            })
            .bind_group_layout(camera_bind_group_layout)
            .format(format)
            .depth_format(depth_format)
            .topology(wgpu::PrimitiveTopology::LineList)
            .label("Grid Pipeline")
            .build();

        let vertices = Self::generate_grid_vertices(10.0, 10);
        let vertex_count = vertices.len() as u32;

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Grid Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        Self {
            pipeline,
            vertex_buffer,
            vertex_count,
            bind_group,
        }
    }

    fn generate_grid_vertices(size: f32, divisions: i32) -> Vec<GridVertex> {
        let mut vertices = Vec::new();
        let half_size = size / 2.0;
        let step = size / divisions as f32;

        let grid_color = [0.3, 0.3, 0.3, 1.0];
        let axis_x_color = [0.8, 0.2, 0.2, 1.0];
        let axis_y_color = [0.2, 0.8, 0.2, 1.0];

        for i in 0..=divisions {
            let pos = -half_size + step * i as f32;
            let color = if i == divisions / 2 {
                axis_y_color
            } else {
                grid_color
            };

            // Lines parallel to Y axis
            vertices.push(GridVertex {
                position: [pos, -half_size, 0.0],
                color,
            });
            vertices.push(GridVertex {
                position: [pos, half_size, 0.0],
                color,
            });

            let color = if i == divisions / 2 {
                axis_x_color
            } else {
                grid_color
            };

            // Lines parallel to X axis
            vertices.push(GridVertex {
                position: [-half_size, pos, 0.0],
                color,
            });
            vertices.push(GridVertex {
                position: [half_size, pos, 0.0],
                color,
            });
        }

        // Z axis
        vertices.push(GridVertex {
            position: [0.0, 0.0, 0.0],
            color: [0.2, 0.2, 0.8, 1.0],
        });
        vertices.push(GridVertex {
            position: [0.0, 0.0, half_size],
            color: [0.2, 0.2, 0.8, 1.0],
        });

        vertices
    }

    pub fn render<'a>(&'a self, render_pass: &mut wgpu::RenderPass<'a>) {
        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_bind_group(0, &self.bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        render_pass.draw(0..self.vertex_count, 0..1);
    }
}
