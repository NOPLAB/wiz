use bytemuck::{Pod, Zeroable};
use glam::Mat4;
use wgpu::util::DeviceExt;

/// Vertex for TF axis lines (position + color)
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct TfAxisVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

/// Instance data for a single TF frame (4x4 transform matrix)
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct TfFrameInstance {
    pub model: [[f32; 4]; 4],
}

impl TfFrameInstance {
    pub fn from_mat4(mat: Mat4) -> Self {
        Self {
            model: mat.to_cols_array_2d(),
        }
    }
}

/// Settings for TF axis rendering
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct TfAxisSettings {
    axis_length: f32,
    line_width: f32,
    _padding: [f32; 2],
}

/// Renderer for TF frame coordinate axes
pub struct TfAxisRenderer {
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
    instance_buffer: wgpu::Buffer,
    instance_count: u32,
    max_instances: u32,
    settings_buffer: wgpu::Buffer,
    settings_bind_group: wgpu::BindGroup,
    camera_bind_group: wgpu::BindGroup,
    axis_length: f32,
}

impl TfAxisRenderer {
    const MAX_FRAMES: u32 = 256;

    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        depth_format: wgpu::TextureFormat,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
        camera_buffer: &wgpu::Buffer,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("TF Axis Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/tf_axis.wgsl").into()),
        });

        let axis_length = 0.5;
        let settings = TfAxisSettings {
            axis_length,
            line_width: 2.0,
            _padding: [0.0; 2],
        };

        let settings_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("TF Axis Settings Buffer"),
            contents: bytemuck::cast_slice(&[settings]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let settings_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("TF Axis Settings Bind Group Layout"),
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
            label: Some("TF Axis Settings Bind Group"),
            layout: &settings_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: settings_buffer.as_entire_binding(),
            }],
        });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("TF Axis Camera Bind Group"),
            layout: camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        // Create axis vertices (3 lines: X=red, Y=green, Z=blue)
        let vertices = Self::create_axis_vertices();
        let vertex_count = vertices.len() as u32;

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("TF Axis Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Create instance buffer for frame transforms
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("TF Axis Instance Buffer"),
            size: (Self::MAX_FRAMES as usize * std::mem::size_of::<TfFrameInstance>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<TfAxisVertex>() as u64,
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
        };

        let instance_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<TfFrameInstance>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: 16,
                    shader_location: 3,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: 32,
                    shader_location: 4,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: 48,
                    shader_location: 5,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        };

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("TF Axis Pipeline Layout"),
            bind_group_layouts: &[camera_bind_group_layout, &settings_bind_group_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("TF Axis Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[vertex_layout, instance_layout],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: depth_format,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self {
            pipeline,
            vertex_buffer,
            vertex_count,
            instance_buffer,
            instance_count: 0,
            max_instances: Self::MAX_FRAMES,
            settings_buffer,
            settings_bind_group,
            camera_bind_group,
            axis_length,
        }
    }

    /// Create axis vertices: 3 lines from origin along X, Y, Z axes
    fn create_axis_vertices() -> Vec<TfAxisVertex> {
        let red = [1.0, 0.2, 0.2, 1.0];
        let green = [0.2, 1.0, 0.2, 1.0];
        let blue = [0.2, 0.2, 1.0, 1.0];

        vec![
            // X axis (red)
            TfAxisVertex {
                position: [0.0, 0.0, 0.0],
                color: red,
            },
            TfAxisVertex {
                position: [1.0, 0.0, 0.0],
                color: red,
            },
            // Y axis (green)
            TfAxisVertex {
                position: [0.0, 0.0, 0.0],
                color: green,
            },
            TfAxisVertex {
                position: [0.0, 1.0, 0.0],
                color: green,
            },
            // Z axis (blue)
            TfAxisVertex {
                position: [0.0, 0.0, 0.0],
                color: blue,
            },
            TfAxisVertex {
                position: [0.0, 0.0, 1.0],
                color: blue,
            },
        ]
    }

    /// Set the length of coordinate axes
    pub fn set_axis_length(&mut self, length: f32) {
        self.axis_length = length;
    }

    /// Update settings uniform buffer
    pub fn update_settings(&self, queue: &wgpu::Queue) {
        let settings = TfAxisSettings {
            axis_length: self.axis_length,
            line_width: 2.0,
            _padding: [0.0; 2],
        };
        queue.write_buffer(&self.settings_buffer, 0, bytemuck::cast_slice(&[settings]));
    }

    /// Update frame instances from transform matrices
    pub fn update_frames(&mut self, queue: &wgpu::Queue, transforms: &[Mat4]) {
        let count = transforms.len().min(self.max_instances as usize);
        self.instance_count = count as u32;

        if count == 0 {
            return;
        }

        let instances: Vec<TfFrameInstance> = transforms
            .iter()
            .take(count)
            .map(|m| TfFrameInstance::from_mat4(*m))
            .collect();

        queue.write_buffer(&self.instance_buffer, 0, bytemuck::cast_slice(&instances));
    }

    /// Render all TF frames
    pub fn render<'a>(&'a self, render_pass: &mut wgpu::RenderPass<'a>) {
        if self.instance_count == 0 {
            return;
        }

        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
        render_pass.set_bind_group(1, &self.settings_bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
        render_pass.draw(0..self.vertex_count, 0..self.instance_count);
    }
}
