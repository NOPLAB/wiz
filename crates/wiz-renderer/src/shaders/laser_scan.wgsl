struct CameraUniform {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    proj: mat4x4<f32>,
    eye: vec4<f32>,
};

struct LaserScanSettings {
    color: vec4<f32>,
    line_width: f32,
    show_points: u32,
    _padding: vec2<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var<uniform> settings: LaserScanSettings;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) intensity: f32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = camera.view_proj * vec4<f32>(in.position, 1.0);

    // Color based on intensity or use default color
    if (in.intensity > 0.0) {
        out.color = vec4<f32>(settings.color.rgb * in.intensity, settings.color.a);
    } else {
        out.color = settings.color;
    }

    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
