// Marker shader for visualization_msgs/Marker rendering
// Supports solid primitives (Cube, Sphere, Cylinder, Arrow) and lines

struct CameraUniform {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    proj: mat4x4<f32>,
    eye: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

// ============================================================================
// Solid Primitives (instanced rendering)
// ============================================================================

struct SolidVertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct SolidInstanceInput {
    @location(2) model_0: vec4<f32>,
    @location(3) model_1: vec4<f32>,
    @location(4) model_2: vec4<f32>,
    @location(5) model_3: vec4<f32>,
    @location(6) color: vec4<f32>,
};

struct SolidVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) world_position: vec3<f32>,
};

@vertex
fn vs_solid(vertex: SolidVertexInput, instance: SolidInstanceInput) -> SolidVertexOutput {
    let model = mat4x4<f32>(
        instance.model_0,
        instance.model_1,
        instance.model_2,
        instance.model_3,
    );

    let world_pos = model * vec4<f32>(vertex.position, 1.0);

    // Transform normal using the upper-left 3x3 of the model matrix
    // Note: For non-uniform scaling, we should use the inverse transpose
    let normal_matrix = mat3x3<f32>(
        model[0].xyz,
        model[1].xyz,
        model[2].xyz,
    );
    let world_normal = normalize(normal_matrix * vertex.normal);

    var out: SolidVertexOutput;
    out.clip_position = camera.view_proj * world_pos;
    out.color = instance.color;
    out.world_normal = world_normal;
    out.world_position = world_pos.xyz;
    return out;
}

@fragment
fn fs_solid(in: SolidVertexOutput) -> @location(0) vec4<f32> {
    // Simple Phong-like lighting for better 3D perception
    let light_dir = normalize(vec3<f32>(0.5, 0.3, 1.0));
    let view_dir = normalize(camera.eye.xyz - in.world_position);
    let half_dir = normalize(light_dir + view_dir);

    // Ambient
    let ambient = 0.3;

    // Diffuse
    let diffuse = max(dot(in.world_normal, light_dir), 0.0) * 0.5;

    // Specular
    let specular = pow(max(dot(in.world_normal, half_dir), 0.0), 32.0) * 0.2;

    let lighting = ambient + diffuse + specular;

    return vec4<f32>(in.color.rgb * lighting, in.color.a);
}

// ============================================================================
// Line rendering
// ============================================================================

struct LineVertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct LineVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_line(vertex: LineVertexInput) -> LineVertexOutput {
    var out: LineVertexOutput;
    out.clip_position = camera.view_proj * vec4<f32>(vertex.position, 1.0);
    out.color = vertex.color;
    return out;
}

@fragment
fn fs_line(in: LineVertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
