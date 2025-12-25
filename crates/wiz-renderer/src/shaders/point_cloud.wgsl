struct CameraUniform {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    proj: mat4x4<f32>,
    eye: vec4<f32>,
};

struct PointCloudSettings {
    point_size: f32,
    alpha: f32,
    _padding: vec2<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var<uniform> settings: PointCloudSettings;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

// Billboard quad offsets for 6 vertices (2 triangles)
const QUAD_VERTICES: array<vec2<f32>, 6> = array<vec2<f32>, 6>(
    vec2<f32>(-0.5, -0.5),
    vec2<f32>( 0.5, -0.5),
    vec2<f32>( 0.5,  0.5),
    vec2<f32>(-0.5, -0.5),
    vec2<f32>( 0.5,  0.5),
    vec2<f32>(-0.5,  0.5),
);

@vertex
fn vs_main(
    in: VertexInput,
    @builtin(vertex_index) vertex_index: u32,
) -> VertexOutput {
    var out: VertexOutput;

    // Get quad vertex offset
    let quad_offset = QUAD_VERTICES[vertex_index % 6u];

    // Transform point to view space
    let view_pos = camera.view * vec4<f32>(in.position, 1.0);

    // Add billboard offset in view space (screen-aligned)
    let size = settings.point_size * 0.01; // Scale factor for world units
    let billboard_pos = view_pos + vec4<f32>(quad_offset.x * size, quad_offset.y * size, 0.0, 0.0);

    // Project to clip space
    out.clip_position = camera.proj * billboard_pos;
    out.color = vec4<f32>(in.color.rgb, in.color.a * settings.alpha);

    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
