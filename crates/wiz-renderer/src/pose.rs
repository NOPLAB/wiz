use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Quat, Vec3};
use wgpu::util::DeviceExt;

/// Vertex for pose arrow (position + color)
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct PoseVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

/// Instance data for a single pose (4x4 transform matrix)
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct PoseInstance {
    pub model: [[f32; 4]; 4],
}

impl PoseInstance {
    pub fn from_mat4(mat: Mat4) -> Self {
        Self {
            model: mat.to_cols_array_2d(),
        }
    }

    /// Create instance from position and quaternion orientation
    pub fn from_pose(position: [f64; 3], orientation: [f64; 4]) -> Self {
        let pos = Vec3::new(position[0] as f32, position[1] as f32, position[2] as f32);
        let quat = Quat::from_xyzw(
            orientation[0] as f32,
            orientation[1] as f32,
            orientation[2] as f32,
            orientation[3] as f32,
        )
        .normalize();
        let mat = Mat4::from_rotation_translation(quat, pos);
        Self::from_mat4(mat)
    }
}

/// Settings for pose arrow rendering
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct PoseSettings {
    arrow_length: f32,
    arrow_width: f32,
    color: [f32; 4],
}

/// Data for a single pose display
pub struct PoseData {
    pub vertex_buffer: wgpu::Buffer,
    pub vertex_count: u32,
    max_vertices: u32,
}

impl PoseData {
    pub fn new(device: &wgpu::Device, vertices: &[PoseVertex]) -> Self {
        const MAX_VERTICES: u32 = 65536;
        let vertex_count = vertices.len() as u32;

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Pose Vertex Buffer"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        });

        Self {
            vertex_buffer,
            vertex_count,
            max_vertices: MAX_VERTICES,
        }
    }

    pub fn update(&mut self, queue: &wgpu::Queue, vertices: &[PoseVertex]) {
        let count = vertices.len().min(self.max_vertices as usize);
        self.vertex_count = count as u32;
        if count > 0 {
            queue.write_buffer(
                &self.vertex_buffer,
                0,
                bytemuck::cast_slice(&vertices[..count]),
            );
        }
    }
}

/// Renderer for PoseStamped messages (arrow visualization)
pub struct PoseRenderer {
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
    instance_buffer: wgpu::Buffer,
    instance_count: u32,
    max_instances: u32,
    settings_buffer: wgpu::Buffer,
    settings_bind_group: wgpu::BindGroup,
    camera_bind_group: wgpu::BindGroup,
    arrow_length: f32,
    arrow_width: f32,
    color: [f32; 4],
}

impl PoseRenderer {
    const MAX_POSES: u32 = 256;

    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        depth_format: wgpu::TextureFormat,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
        camera_buffer: &wgpu::Buffer,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Pose Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/pose.wgsl").into()),
        });

        let arrow_length = 1.0;
        let arrow_width = 0.1;
        let color = [1.0, 0.0, 0.5, 1.0]; // Magenta by default

        let settings = PoseSettings {
            arrow_length,
            arrow_width,
            color,
        };

        let settings_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Pose Settings Buffer"),
            contents: bytemuck::cast_slice(&[settings]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let settings_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Pose Settings Bind Group Layout"),
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
            label: Some("Pose Settings Bind Group"),
            layout: &settings_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: settings_buffer.as_entire_binding(),
            }],
        });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Pose Camera Bind Group"),
            layout: camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        // Create arrow vertices (shaft + head)
        let vertices = Self::create_arrow_vertices();
        let vertex_count = vertices.len() as u32;

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Pose Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Create instance buffer for pose transforms
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Pose Instance Buffer"),
            size: (Self::MAX_POSES as usize * std::mem::size_of::<PoseInstance>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<PoseVertex>() as u64,
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
            array_stride: std::mem::size_of::<PoseInstance>() as u64,
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
            label: Some("Pose Pipeline Layout"),
            bind_group_layouts: &[camera_bind_group_layout, &settings_bind_group_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Pose Pipeline"),
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
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
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
            max_instances: Self::MAX_POSES,
            settings_buffer,
            settings_bind_group,
            camera_bind_group,
            arrow_length,
            arrow_width,
            color,
        }
    }

    /// Create arrow vertices: shaft (box) + head (cone)
    /// Arrow points along positive X axis (standard ROS convention)
    fn create_arrow_vertices() -> Vec<PoseVertex> {
        let mut vertices = Vec::new();

        // Shaft parameters (relative to arrow_length=1)
        let shaft_length = 0.7;
        let shaft_radius = 0.05;

        // Head parameters
        let head_length = 0.3;
        let head_radius = 0.12;

        let shaft_color = [1.0, 1.0, 1.0, 1.0]; // Will be multiplied by settings.color
        let head_color = [1.0, 1.0, 1.0, 1.0];

        // Create shaft as a hexagonal prism (6 sides)
        let n_sides = 6;
        for i in 0..n_sides {
            let angle1 = (i as f32) * std::f32::consts::TAU / n_sides as f32;
            let angle2 = ((i + 1) % n_sides) as f32 * std::f32::consts::TAU / n_sides as f32;

            let y1 = shaft_radius * angle1.cos();
            let z1 = shaft_radius * angle1.sin();
            let y2 = shaft_radius * angle2.cos();
            let z2 = shaft_radius * angle2.sin();

            // Front face (at x=0)
            vertices.push(PoseVertex {
                position: [0.0, 0.0, 0.0],
                color: shaft_color,
            });
            vertices.push(PoseVertex {
                position: [0.0, y1, z1],
                color: shaft_color,
            });
            vertices.push(PoseVertex {
                position: [0.0, y2, z2],
                color: shaft_color,
            });

            // Back face (at x=shaft_length)
            vertices.push(PoseVertex {
                position: [shaft_length, 0.0, 0.0],
                color: shaft_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y2, z2],
                color: shaft_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y1, z1],
                color: shaft_color,
            });

            // Side faces
            vertices.push(PoseVertex {
                position: [0.0, y1, z1],
                color: shaft_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y1, z1],
                color: shaft_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y2, z2],
                color: shaft_color,
            });

            vertices.push(PoseVertex {
                position: [0.0, y1, z1],
                color: shaft_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y2, z2],
                color: shaft_color,
            });
            vertices.push(PoseVertex {
                position: [0.0, y2, z2],
                color: shaft_color,
            });
        }

        // Create head as a cone
        for i in 0..n_sides {
            let angle1 = (i as f32) * std::f32::consts::TAU / n_sides as f32;
            let angle2 = ((i + 1) % n_sides) as f32 * std::f32::consts::TAU / n_sides as f32;

            let y1 = head_radius * angle1.cos();
            let z1 = head_radius * angle1.sin();
            let y2 = head_radius * angle2.cos();
            let z2 = head_radius * angle2.sin();

            // Cone tip
            vertices.push(PoseVertex {
                position: [shaft_length + head_length, 0.0, 0.0],
                color: head_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y2, z2],
                color: head_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y1, z1],
                color: head_color,
            });

            // Cone base
            vertices.push(PoseVertex {
                position: [shaft_length, 0.0, 0.0],
                color: head_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y1, z1],
                color: head_color,
            });
            vertices.push(PoseVertex {
                position: [shaft_length, y2, z2],
                color: head_color,
            });
        }

        vertices
    }

    /// Set the arrow length
    pub fn set_arrow_length(&mut self, length: f32) {
        self.arrow_length = length;
    }

    /// Set the arrow width (scale factor)
    pub fn set_arrow_width(&mut self, width: f32) {
        self.arrow_width = width;
    }

    /// Set the arrow color
    pub fn set_color(&mut self, r: f32, g: f32, b: f32, a: f32) {
        self.color = [r, g, b, a];
    }

    /// Update settings uniform buffer
    pub fn update_settings(&self, queue: &wgpu::Queue) {
        let settings = PoseSettings {
            arrow_length: self.arrow_length,
            arrow_width: self.arrow_width,
            color: self.color,
        };
        queue.write_buffer(&self.settings_buffer, 0, bytemuck::cast_slice(&[settings]));
    }

    /// Update pose instances from PoseInstance data
    pub fn update_poses(&mut self, queue: &wgpu::Queue, poses: &[PoseInstance]) {
        let count = poses.len().min(self.max_instances as usize);
        self.instance_count = count as u32;

        if count == 0 {
            return;
        }

        queue.write_buffer(
            &self.instance_buffer,
            0,
            bytemuck::cast_slice(&poses[..count]),
        );
    }

    /// Update poses from transform matrices
    pub fn update_from_matrices(&mut self, queue: &wgpu::Queue, transforms: &[Mat4]) {
        let instances: Vec<PoseInstance> = transforms
            .iter()
            .map(|m| PoseInstance::from_mat4(*m))
            .collect();
        self.update_poses(queue, &instances);
    }

    /// Render all poses
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
