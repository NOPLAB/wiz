use wgpu::{
    BindGroupLayout, ColorTargetState, Device, RenderPipeline, ShaderModule, TextureFormat,
    VertexBufferLayout,
};

pub struct PipelineBuilder<'a> {
    device: &'a Device,
    shader: Option<&'a ShaderModule>,
    vertex_layouts: Vec<VertexBufferLayout<'a>>,
    bind_group_layouts: Vec<&'a BindGroupLayout>,
    format: TextureFormat,
    depth_format: Option<TextureFormat>,
    label: &'a str,
    topology: wgpu::PrimitiveTopology,
}

impl<'a> PipelineBuilder<'a> {
    pub fn new(device: &'a Device) -> Self {
        Self {
            device,
            shader: None,
            vertex_layouts: Vec::new(),
            bind_group_layouts: Vec::new(),
            format: TextureFormat::Bgra8UnormSrgb,
            depth_format: None,
            label: "Pipeline",
            topology: wgpu::PrimitiveTopology::TriangleList,
        }
    }

    pub fn shader(mut self, shader: &'a ShaderModule) -> Self {
        self.shader = Some(shader);
        self
    }

    pub fn vertex_layout(mut self, layout: VertexBufferLayout<'a>) -> Self {
        self.vertex_layouts.push(layout);
        self
    }

    pub fn bind_group_layout(mut self, layout: &'a BindGroupLayout) -> Self {
        self.bind_group_layouts.push(layout);
        self
    }

    pub fn format(mut self, format: TextureFormat) -> Self {
        self.format = format;
        self
    }

    pub fn depth_format(mut self, format: TextureFormat) -> Self {
        self.depth_format = Some(format);
        self
    }

    pub fn label(mut self, label: &'a str) -> Self {
        self.label = label;
        self
    }

    pub fn topology(mut self, topology: wgpu::PrimitiveTopology) -> Self {
        self.topology = topology;
        self
    }

    pub fn build(self) -> RenderPipeline {
        let shader = self.shader.expect("Shader is required");

        let pipeline_layout = self
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some(&format!("{} Layout", self.label)),
                bind_group_layouts: &self.bind_group_layouts,
                push_constant_ranges: &[],
            });

        let depth_stencil = self.depth_format.map(|format| wgpu::DepthStencilState {
            format,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        });

        self.device
            .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(self.label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: shader,
                    entry_point: "vs_main",
                    buffers: &self.vertex_layouts,
                    compilation_options: Default::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module: shader,
                    entry_point: "fs_main",
                    targets: &[Some(ColorTargetState {
                        format: self.format,
                        blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: Default::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology: self.topology,
                    strip_index_format: None,
                    front_face: wgpu::FrontFace::Ccw,
                    cull_mode: None,
                    polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false,
                    conservative: false,
                },
                depth_stencil,
                multisample: wgpu::MultisampleState::default(),
                multiview: None,
                cache: None,
            })
    }
}
