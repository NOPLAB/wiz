//! Marker renderer for visualization_msgs/Marker and MarkerArray
//!
//! Supports rendering various marker types using GPU instancing:
//! - Arrow, Cube, Sphere, Cylinder
//! - LineStrip, LineList
//! - Points, CubeList, SphereList

use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Quat, Vec3};
use std::collections::HashMap;
use wgpu::util::DeviceExt;
use wiz_core::{Marker, MarkerAction, MarkerType};

/// Vertex for marker primitives (position + normal)
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct MarkerVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

/// Instance data for a single marker
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct MarkerInstance {
    pub model: [[f32; 4]; 4],
    pub color: [f32; 4],
}

impl MarkerInstance {
    pub fn from_marker(marker: &Marker) -> Self {
        let pos = Vec3::new(
            marker.pose.position[0] as f32,
            marker.pose.position[1] as f32,
            marker.pose.position[2] as f32,
        );
        let quat = Quat::from_xyzw(
            marker.pose.orientation[0] as f32,
            marker.pose.orientation[1] as f32,
            marker.pose.orientation[2] as f32,
            marker.pose.orientation[3] as f32,
        )
        .normalize();
        let scale = Vec3::new(
            marker.scale.x as f32,
            marker.scale.y as f32,
            marker.scale.z as f32,
        );

        let mat = Mat4::from_scale_rotation_translation(scale, quat, pos);

        Self {
            model: mat.to_cols_array_2d(),
            color: [
                marker.color.r,
                marker.color.g,
                marker.color.b,
                marker.color.a,
            ],
        }
    }
}

/// Line vertex for LineStrip and LineList
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct LineVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

/// Key for marker identification (namespace + id)
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct MarkerKey {
    pub ns: String,
    pub id: i32,
}

impl From<&Marker> for MarkerKey {
    fn from(m: &Marker) -> Self {
        Self {
            ns: m.ns.clone(),
            id: m.id,
        }
    }
}

/// Stored marker with rendering data
struct StoredMarker {
    marker: Marker,
    instance: MarkerInstance,
}

/// Primitive mesh data
struct PrimitiveMesh {
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
}

/// Renderer for visualization_msgs/Marker
pub struct MarkerRenderer {
    // Pipelines
    solid_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,

    // Primitive meshes
    cube_mesh: PrimitiveMesh,
    sphere_mesh: PrimitiveMesh,
    cylinder_mesh: PrimitiveMesh,
    arrow_mesh: PrimitiveMesh,

    // Instance buffer (shared across all solid primitives)
    instance_buffer: wgpu::Buffer,
    max_instances: u32,

    // Line buffer
    line_buffer: wgpu::Buffer,
    line_vertex_count: u32,
    max_line_vertices: u32,

    // Bind groups
    camera_bind_group: wgpu::BindGroup,

    // Marker storage by type
    markers: HashMap<MarkerKey, StoredMarker>,

    // Instances grouped by marker type for batched rendering
    cube_instances: Vec<MarkerInstance>,
    sphere_instances: Vec<MarkerInstance>,
    cylinder_instances: Vec<MarkerInstance>,
    arrow_instances: Vec<MarkerInstance>,
    line_vertices: Vec<LineVertex>,

    // Dirty flag
    dirty: bool,
}

impl MarkerRenderer {
    const MAX_INSTANCES: u32 = 1024;
    const MAX_LINE_VERTICES: u32 = 65536;

    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        depth_format: wgpu::TextureFormat,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
        camera_buffer: &wgpu::Buffer,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Marker Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/marker.wgsl").into()),
        });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Marker Camera Bind Group"),
            layout: camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        // Create primitive meshes
        let cube_mesh = Self::create_cube_mesh(device);
        let sphere_mesh = Self::create_sphere_mesh(device, 16, 12);
        let cylinder_mesh = Self::create_cylinder_mesh(device, 16);
        let arrow_mesh = Self::create_arrow_mesh(device);

        // Create instance buffer
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Marker Instance Buffer"),
            size: (Self::MAX_INSTANCES as usize * std::mem::size_of::<MarkerInstance>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Create line buffer
        let line_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Marker Line Buffer"),
            size: (Self::MAX_LINE_VERTICES as usize * std::mem::size_of::<LineVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Vertex layouts
        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<MarkerVertex>() as u64,
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
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        };

        let instance_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<MarkerInstance>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &[
                // Model matrix (4 vec4s)
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
                // Color
                wgpu::VertexAttribute {
                    offset: 64,
                    shader_location: 6,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        };

        let line_vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<LineVertex>() as u64,
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

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Marker Pipeline Layout"),
            bind_group_layouts: &[camera_bind_group_layout],
            push_constant_ranges: &[],
        });

        // Solid pipeline for 3D primitives
        let solid_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Marker Solid Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_solid"),
                buffers: &[vertex_layout, instance_layout],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_solid"),
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

        // Line pipeline
        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Marker Line Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_line"),
                buffers: &[line_vertex_layout],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_line"),
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
            solid_pipeline,
            line_pipeline,
            cube_mesh,
            sphere_mesh,
            cylinder_mesh,
            arrow_mesh,
            instance_buffer,
            max_instances: Self::MAX_INSTANCES,
            line_buffer,
            line_vertex_count: 0,
            max_line_vertices: Self::MAX_LINE_VERTICES,
            camera_bind_group,
            markers: HashMap::new(),
            cube_instances: Vec::new(),
            sphere_instances: Vec::new(),
            cylinder_instances: Vec::new(),
            arrow_instances: Vec::new(),
            line_vertices: Vec::new(),
            dirty: false,
        }
    }

    /// Create a unit cube mesh centered at origin
    fn create_cube_mesh(device: &wgpu::Device) -> PrimitiveMesh {
        let mut vertices = Vec::new();

        // Each face has 2 triangles (6 vertices)
        let faces = [
            // Front (+Z)
            (
                [
                    [-0.5, -0.5, 0.5],
                    [0.5, -0.5, 0.5],
                    [0.5, 0.5, 0.5],
                    [-0.5, 0.5, 0.5],
                ],
                [0.0, 0.0, 1.0],
            ),
            // Back (-Z)
            (
                [
                    [0.5, -0.5, -0.5],
                    [-0.5, -0.5, -0.5],
                    [-0.5, 0.5, -0.5],
                    [0.5, 0.5, -0.5],
                ],
                [0.0, 0.0, -1.0],
            ),
            // Right (+X)
            (
                [
                    [0.5, -0.5, 0.5],
                    [0.5, -0.5, -0.5],
                    [0.5, 0.5, -0.5],
                    [0.5, 0.5, 0.5],
                ],
                [1.0, 0.0, 0.0],
            ),
            // Left (-X)
            (
                [
                    [-0.5, -0.5, -0.5],
                    [-0.5, -0.5, 0.5],
                    [-0.5, 0.5, 0.5],
                    [-0.5, 0.5, -0.5],
                ],
                [-1.0, 0.0, 0.0],
            ),
            // Top (+Y)
            (
                [
                    [-0.5, 0.5, 0.5],
                    [0.5, 0.5, 0.5],
                    [0.5, 0.5, -0.5],
                    [-0.5, 0.5, -0.5],
                ],
                [0.0, 1.0, 0.0],
            ),
            // Bottom (-Y)
            (
                [
                    [-0.5, -0.5, -0.5],
                    [0.5, -0.5, -0.5],
                    [0.5, -0.5, 0.5],
                    [-0.5, -0.5, 0.5],
                ],
                [0.0, -1.0, 0.0],
            ),
        ];

        for (quad, normal) in faces {
            // First triangle
            vertices.push(MarkerVertex {
                position: quad[0],
                normal,
            });
            vertices.push(MarkerVertex {
                position: quad[1],
                normal,
            });
            vertices.push(MarkerVertex {
                position: quad[2],
                normal,
            });
            // Second triangle
            vertices.push(MarkerVertex {
                position: quad[0],
                normal,
            });
            vertices.push(MarkerVertex {
                position: quad[2],
                normal,
            });
            vertices.push(MarkerVertex {
                position: quad[3],
                normal,
            });
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Cube Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        PrimitiveMesh {
            vertex_buffer,
            vertex_count: vertices.len() as u32,
        }
    }

    /// Create a UV sphere mesh
    fn create_sphere_mesh(device: &wgpu::Device, segments: u32, rings: u32) -> PrimitiveMesh {
        let mut vertices = Vec::new();

        for ring in 0..rings {
            let theta1 = (ring as f32 / rings as f32) * std::f32::consts::PI;
            let theta2 = ((ring + 1) as f32 / rings as f32) * std::f32::consts::PI;

            for seg in 0..segments {
                let phi1 = (seg as f32 / segments as f32) * std::f32::consts::TAU;
                let phi2 = ((seg + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                // Four corners of the quad
                let p1 = [
                    0.5 * theta1.sin() * phi1.cos(),
                    0.5 * theta1.cos(),
                    0.5 * theta1.sin() * phi1.sin(),
                ];
                let p2 = [
                    0.5 * theta1.sin() * phi2.cos(),
                    0.5 * theta1.cos(),
                    0.5 * theta1.sin() * phi2.sin(),
                ];
                let p3 = [
                    0.5 * theta2.sin() * phi2.cos(),
                    0.5 * theta2.cos(),
                    0.5 * theta2.sin() * phi2.sin(),
                ];
                let p4 = [
                    0.5 * theta2.sin() * phi1.cos(),
                    0.5 * theta2.cos(),
                    0.5 * theta2.sin() * phi1.sin(),
                ];

                // Normals (same as positions for unit sphere, but normalized)
                let n1 = Self::normalize(p1);
                let n2 = Self::normalize(p2);
                let n3 = Self::normalize(p3);
                let n4 = Self::normalize(p4);

                // First triangle
                vertices.push(MarkerVertex {
                    position: p1,
                    normal: n1,
                });
                vertices.push(MarkerVertex {
                    position: p2,
                    normal: n2,
                });
                vertices.push(MarkerVertex {
                    position: p3,
                    normal: n3,
                });

                // Second triangle
                vertices.push(MarkerVertex {
                    position: p1,
                    normal: n1,
                });
                vertices.push(MarkerVertex {
                    position: p3,
                    normal: n3,
                });
                vertices.push(MarkerVertex {
                    position: p4,
                    normal: n4,
                });
            }
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Sphere Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        PrimitiveMesh {
            vertex_buffer,
            vertex_count: vertices.len() as u32,
        }
    }

    /// Create a cylinder mesh along the Z axis
    fn create_cylinder_mesh(device: &wgpu::Device, segments: u32) -> PrimitiveMesh {
        let mut vertices = Vec::new();
        let radius = 0.5;
        let half_height = 0.5;

        for i in 0..segments {
            let angle1 = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let angle2 = ((i + 1) as f32 / segments as f32) * std::f32::consts::TAU;

            let x1 = radius * angle1.cos();
            let y1 = radius * angle1.sin();
            let x2 = radius * angle2.cos();
            let y2 = radius * angle2.sin();

            let n1 = Self::normalize([x1, y1, 0.0]);
            let n2 = Self::normalize([x2, y2, 0.0]);

            // Side face
            vertices.push(MarkerVertex {
                position: [x1, y1, -half_height],
                normal: n1,
            });
            vertices.push(MarkerVertex {
                position: [x2, y2, -half_height],
                normal: n2,
            });
            vertices.push(MarkerVertex {
                position: [x2, y2, half_height],
                normal: n2,
            });

            vertices.push(MarkerVertex {
                position: [x1, y1, -half_height],
                normal: n1,
            });
            vertices.push(MarkerVertex {
                position: [x2, y2, half_height],
                normal: n2,
            });
            vertices.push(MarkerVertex {
                position: [x1, y1, half_height],
                normal: n1,
            });

            // Top cap
            vertices.push(MarkerVertex {
                position: [0.0, 0.0, half_height],
                normal: [0.0, 0.0, 1.0],
            });
            vertices.push(MarkerVertex {
                position: [x1, y1, half_height],
                normal: [0.0, 0.0, 1.0],
            });
            vertices.push(MarkerVertex {
                position: [x2, y2, half_height],
                normal: [0.0, 0.0, 1.0],
            });

            // Bottom cap
            vertices.push(MarkerVertex {
                position: [0.0, 0.0, -half_height],
                normal: [0.0, 0.0, -1.0],
            });
            vertices.push(MarkerVertex {
                position: [x2, y2, -half_height],
                normal: [0.0, 0.0, -1.0],
            });
            vertices.push(MarkerVertex {
                position: [x1, y1, -half_height],
                normal: [0.0, 0.0, -1.0],
            });
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Cylinder Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        PrimitiveMesh {
            vertex_buffer,
            vertex_count: vertices.len() as u32,
        }
    }

    /// Create an arrow mesh pointing along +X axis
    fn create_arrow_mesh(device: &wgpu::Device) -> PrimitiveMesh {
        let mut vertices = Vec::new();
        let n_sides = 8;

        // Shaft parameters
        let shaft_length = 0.7;
        let shaft_radius = 0.05;

        // Head parameters
        let head_length = 0.3;
        let head_radius = 0.12;

        // Create shaft
        for i in 0..n_sides {
            let angle1 = (i as f32) * std::f32::consts::TAU / n_sides as f32;
            let angle2 = ((i + 1) % n_sides) as f32 * std::f32::consts::TAU / n_sides as f32;

            let y1 = shaft_radius * angle1.cos();
            let z1 = shaft_radius * angle1.sin();
            let y2 = shaft_radius * angle2.cos();
            let z2 = shaft_radius * angle2.sin();

            let n1 = Self::normalize([0.0, angle1.cos(), angle1.sin()]);
            let n2 = Self::normalize([0.0, angle2.cos(), angle2.sin()]);

            // Side face
            vertices.push(MarkerVertex {
                position: [0.0, y1, z1],
                normal: n1,
            });
            vertices.push(MarkerVertex {
                position: [shaft_length, y1, z1],
                normal: n1,
            });
            vertices.push(MarkerVertex {
                position: [shaft_length, y2, z2],
                normal: n2,
            });

            vertices.push(MarkerVertex {
                position: [0.0, y1, z1],
                normal: n1,
            });
            vertices.push(MarkerVertex {
                position: [shaft_length, y2, z2],
                normal: n2,
            });
            vertices.push(MarkerVertex {
                position: [0.0, y2, z2],
                normal: n2,
            });
        }

        // Create cone head
        for i in 0..n_sides {
            let angle1 = (i as f32) * std::f32::consts::TAU / n_sides as f32;
            let angle2 = ((i + 1) % n_sides) as f32 * std::f32::consts::TAU / n_sides as f32;

            let y1 = head_radius * angle1.cos();
            let z1 = head_radius * angle1.sin();
            let y2 = head_radius * angle2.cos();
            let z2 = head_radius * angle2.sin();

            // Cone normal (angled)
            let cone_angle = (head_radius / head_length).atan();
            let nx = cone_angle.sin();
            let scale = cone_angle.cos();
            let n1 = Self::normalize([nx, scale * angle1.cos(), scale * angle1.sin()]);
            let n2 = Self::normalize([nx, scale * angle2.cos(), scale * angle2.sin()]);

            // Cone tip
            vertices.push(MarkerVertex {
                position: [shaft_length + head_length, 0.0, 0.0],
                normal: [1.0, 0.0, 0.0],
            });
            vertices.push(MarkerVertex {
                position: [shaft_length, y2, z2],
                normal: n2,
            });
            vertices.push(MarkerVertex {
                position: [shaft_length, y1, z1],
                normal: n1,
            });

            // Cone base
            vertices.push(MarkerVertex {
                position: [shaft_length, 0.0, 0.0],
                normal: [-1.0, 0.0, 0.0],
            });
            vertices.push(MarkerVertex {
                position: [shaft_length, y1, z1],
                normal: [-1.0, 0.0, 0.0],
            });
            vertices.push(MarkerVertex {
                position: [shaft_length, y2, z2],
                normal: [-1.0, 0.0, 0.0],
            });
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Arrow Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        PrimitiveMesh {
            vertex_buffer,
            vertex_count: vertices.len() as u32,
        }
    }

    fn normalize(v: [f32; 3]) -> [f32; 3] {
        let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if len > 0.0 {
            [v[0] / len, v[1] / len, v[2] / len]
        } else {
            [0.0, 0.0, 1.0]
        }
    }

    /// Process a marker message (add, modify, or delete)
    pub fn process_marker(&mut self, marker: &Marker) {
        let key = MarkerKey::from(marker);

        match marker.action {
            MarkerAction::Add | MarkerAction::Modify => {
                let instance = MarkerInstance::from_marker(marker);
                self.markers.insert(
                    key,
                    StoredMarker {
                        marker: marker.clone(),
                        instance,
                    },
                );
                self.dirty = true;
            }
            MarkerAction::Delete => {
                self.markers.remove(&key);
                self.dirty = true;
            }
            MarkerAction::DeleteAll => {
                // Delete all markers with matching namespace (or all if ns is empty)
                if marker.ns.is_empty() {
                    self.markers.clear();
                } else {
                    self.markers.retain(|k, _| k.ns != marker.ns);
                }
                self.dirty = true;
            }
        }
    }

    /// Process multiple markers (MarkerArray)
    pub fn process_markers(&mut self, markers: &[Marker]) {
        for marker in markers {
            self.process_marker(marker);
        }
    }

    /// Clear all markers
    pub fn clear(&mut self) {
        self.markers.clear();
        self.dirty = true;
    }

    /// Rebuild instance buffers from stored markers
    fn rebuild_instances(&mut self) {
        self.cube_instances.clear();
        self.sphere_instances.clear();
        self.cylinder_instances.clear();
        self.arrow_instances.clear();
        self.line_vertices.clear();

        // Collect all marker data first to avoid borrow conflicts
        let markers_data: Vec<_> = self
            .markers
            .values()
            .map(|stored| (stored.marker.clone(), stored.instance))
            .collect();

        for (marker, instance) in markers_data {
            match marker.marker_type {
                MarkerType::Cube => {
                    self.cube_instances.push(instance);
                }
                MarkerType::Sphere => {
                    self.sphere_instances.push(instance);
                }
                MarkerType::Cylinder => {
                    self.cylinder_instances.push(instance);
                }
                MarkerType::Arrow => {
                    self.arrow_instances.push(instance);
                }
                MarkerType::LineStrip => {
                    Self::add_line_strip_to(&marker, &mut self.line_vertices);
                }
                MarkerType::LineList => {
                    Self::add_line_list_to(&marker, &mut self.line_vertices);
                }
                MarkerType::CubeList => {
                    Self::add_cube_list_to(&marker, &mut self.cube_instances);
                }
                MarkerType::SphereList => {
                    Self::add_sphere_list_to(&marker, &mut self.sphere_instances);
                }
                MarkerType::Points => {
                    // Points rendered as small cubes
                    Self::add_points_as_cubes_to(&marker, &mut self.cube_instances);
                }
                _ => {
                    // TextViewFacing, MeshResource, TriangleList not yet supported
                }
            }
        }

        self.dirty = false;
    }

    fn add_line_strip_to(marker: &Marker, line_vertices: &mut Vec<LineVertex>) {
        if marker.points.len() < 2 {
            return;
        }

        let default_color = [
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
        ];

        for i in 0..marker.points.len() - 1 {
            let p1 = marker.points[i];
            let p2 = marker.points[i + 1];

            let c1 = if i < marker.colors.len() {
                let c = &marker.colors[i];
                [c.r, c.g, c.b, c.a]
            } else {
                default_color
            };

            let c2 = if i + 1 < marker.colors.len() {
                let c = &marker.colors[i + 1];
                [c.r, c.g, c.b, c.a]
            } else {
                default_color
            };

            line_vertices.push(LineVertex {
                position: [p1[0] as f32, p1[1] as f32, p1[2] as f32],
                color: c1,
            });
            line_vertices.push(LineVertex {
                position: [p2[0] as f32, p2[1] as f32, p2[2] as f32],
                color: c2,
            });
        }
    }

    fn add_line_list_to(marker: &Marker, line_vertices: &mut Vec<LineVertex>) {
        if marker.points.len() < 2 {
            return;
        }

        let default_color = [
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
        ];

        for i in (0..marker.points.len() - 1).step_by(2) {
            let p1 = marker.points[i];
            let p2 = marker.points[i + 1];

            let c1 = if i < marker.colors.len() {
                let c = &marker.colors[i];
                [c.r, c.g, c.b, c.a]
            } else {
                default_color
            };

            let c2 = if i + 1 < marker.colors.len() {
                let c = &marker.colors[i + 1];
                [c.r, c.g, c.b, c.a]
            } else {
                default_color
            };

            line_vertices.push(LineVertex {
                position: [p1[0] as f32, p1[1] as f32, p1[2] as f32],
                color: c1,
            });
            line_vertices.push(LineVertex {
                position: [p2[0] as f32, p2[1] as f32, p2[2] as f32],
                color: c2,
            });
        }
    }

    fn add_cube_list_to(marker: &Marker, cube_instances: &mut Vec<MarkerInstance>) {
        let default_color = [
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
        ];
        let scale = Vec3::new(
            marker.scale.x as f32,
            marker.scale.y as f32,
            marker.scale.z as f32,
        );

        for (i, point) in marker.points.iter().enumerate() {
            let pos = Vec3::new(point[0] as f32, point[1] as f32, point[2] as f32);
            let mat = Mat4::from_scale_rotation_translation(scale, Quat::IDENTITY, pos);

            let color = if i < marker.colors.len() {
                let c = &marker.colors[i];
                [c.r, c.g, c.b, c.a]
            } else {
                default_color
            };

            cube_instances.push(MarkerInstance {
                model: mat.to_cols_array_2d(),
                color,
            });
        }
    }

    fn add_sphere_list_to(marker: &Marker, sphere_instances: &mut Vec<MarkerInstance>) {
        let default_color = [
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
        ];
        let scale = Vec3::new(
            marker.scale.x as f32,
            marker.scale.y as f32,
            marker.scale.z as f32,
        );

        for (i, point) in marker.points.iter().enumerate() {
            let pos = Vec3::new(point[0] as f32, point[1] as f32, point[2] as f32);
            let mat = Mat4::from_scale_rotation_translation(scale, Quat::IDENTITY, pos);

            let color = if i < marker.colors.len() {
                let c = &marker.colors[i];
                [c.r, c.g, c.b, c.a]
            } else {
                default_color
            };

            sphere_instances.push(MarkerInstance {
                model: mat.to_cols_array_2d(),
                color,
            });
        }
    }

    fn add_points_as_cubes_to(marker: &Marker, cube_instances: &mut Vec<MarkerInstance>) {
        let default_color = [
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
        ];
        // Points use scale.x and scale.y for width and height
        let size = marker.scale.x as f32 * 0.1; // Scale down for point rendering
        let scale = Vec3::splat(size);

        for (i, point) in marker.points.iter().enumerate() {
            let pos = Vec3::new(point[0] as f32, point[1] as f32, point[2] as f32);
            let mat = Mat4::from_scale_rotation_translation(scale, Quat::IDENTITY, pos);

            let color = if i < marker.colors.len() {
                let c = &marker.colors[i];
                [c.r, c.g, c.b, c.a]
            } else {
                default_color
            };

            cube_instances.push(MarkerInstance {
                model: mat.to_cols_array_2d(),
                color,
            });
        }
    }

    /// Update GPU buffers
    pub fn update(&mut self, queue: &wgpu::Queue) {
        if self.dirty {
            self.rebuild_instances();
        }

        // Update line buffer
        let line_count = self
            .line_vertices
            .len()
            .min(self.max_line_vertices as usize);
        self.line_vertex_count = line_count as u32;
        if line_count > 0 {
            queue.write_buffer(
                &self.line_buffer,
                0,
                bytemuck::cast_slice(&self.line_vertices[..line_count]),
            );
        }
    }

    /// Render all markers
    pub fn render<'a>(&'a self, render_pass: &mut wgpu::RenderPass<'a>, queue: &wgpu::Queue) {
        // Render solid primitives
        render_pass.set_pipeline(&self.solid_pipeline);
        render_pass.set_bind_group(0, &self.camera_bind_group, &[]);

        // Cubes
        if !self.cube_instances.is_empty() {
            let count = self.cube_instances.len().min(self.max_instances as usize);
            queue.write_buffer(
                &self.instance_buffer,
                0,
                bytemuck::cast_slice(&self.cube_instances[..count]),
            );
            render_pass.set_vertex_buffer(0, self.cube_mesh.vertex_buffer.slice(..));
            render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
            render_pass.draw(0..self.cube_mesh.vertex_count, 0..count as u32);
        }

        // Spheres
        if !self.sphere_instances.is_empty() {
            let count = self.sphere_instances.len().min(self.max_instances as usize);
            queue.write_buffer(
                &self.instance_buffer,
                0,
                bytemuck::cast_slice(&self.sphere_instances[..count]),
            );
            render_pass.set_vertex_buffer(0, self.sphere_mesh.vertex_buffer.slice(..));
            render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
            render_pass.draw(0..self.sphere_mesh.vertex_count, 0..count as u32);
        }

        // Cylinders
        if !self.cylinder_instances.is_empty() {
            let count = self
                .cylinder_instances
                .len()
                .min(self.max_instances as usize);
            queue.write_buffer(
                &self.instance_buffer,
                0,
                bytemuck::cast_slice(&self.cylinder_instances[..count]),
            );
            render_pass.set_vertex_buffer(0, self.cylinder_mesh.vertex_buffer.slice(..));
            render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
            render_pass.draw(0..self.cylinder_mesh.vertex_count, 0..count as u32);
        }

        // Arrows
        if !self.arrow_instances.is_empty() {
            let count = self.arrow_instances.len().min(self.max_instances as usize);
            queue.write_buffer(
                &self.instance_buffer,
                0,
                bytemuck::cast_slice(&self.arrow_instances[..count]),
            );
            render_pass.set_vertex_buffer(0, self.arrow_mesh.vertex_buffer.slice(..));
            render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
            render_pass.draw(0..self.arrow_mesh.vertex_count, 0..count as u32);
        }

        // Render lines
        if self.line_vertex_count > 0 {
            render_pass.set_pipeline(&self.line_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.line_buffer.slice(..));
            render_pass.draw(0..self.line_vertex_count, 0..1);
        }
    }

    /// Get the number of active markers
    pub fn marker_count(&self) -> usize {
        self.markers.len()
    }
}
