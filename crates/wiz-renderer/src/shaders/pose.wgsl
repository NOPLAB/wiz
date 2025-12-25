struct CameraUniform {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    proj: mat4x4<f32>,
    eye: vec4<f32>,
};

struct PoseSettings {
    arrow_length: f32,
    arrow_width: f32,
    color: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var<uniform> settings: PoseSettings;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct InstanceInput {
    @location(2) model_0: vec4<f32>,
    @location(3) model_1: vec4<f32>,
    @location(4) model_2: vec4<f32>,
    @location(5) model_3: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
};

@vertex
fn vs_main(vertex: VertexInput, instance: InstanceInput) -> VertexOutput {
    let model = mat4x4<f32>(
        instance.model_0,
        instance.model_1,
        instance.model_2,
        instance.model_3,
    );

    // Scale position by arrow dimensions
    var scaled_pos = vertex.position;
    scaled_pos.x = scaled_pos.x * settings.arrow_length;
    scaled_pos.y = scaled_pos.y * settings.arrow_width;
    scaled_pos.z = scaled_pos.z * settings.arrow_width;

    let world_pos = model * vec4<f32>(scaled_pos, 1.0);

    // Compute approximate normal for lighting
    let normal = normalize(vertex.position - vec3<f32>(vertex.position.x, 0.0, 0.0));
    let world_normal = normalize((model * vec4<f32>(normal, 0.0)).xyz);

    var out: VertexOutput;
    out.clip_position = camera.view_proj * world_pos;
    out.color = vertex.color * settings.color;
    out.world_normal = world_normal;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Simple lighting for better 3D perception
    let light_dir = normalize(vec3<f32>(0.5, 0.5, 1.0));
    let ambient = 0.4;
    let diffuse = max(dot(in.world_normal, light_dir), 0.0) * 0.6;
    let lighting = ambient + diffuse;

    return vec4<f32>(in.color.rgb * lighting, in.color.a);
}
