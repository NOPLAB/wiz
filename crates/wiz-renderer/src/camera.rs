use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec3};

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct CameraUniform {
    pub view_proj: [[f32; 4]; 4],
    pub view: [[f32; 4]; 4],
    pub proj: [[f32; 4]; 4],
    pub eye: [f32; 4],
}

pub struct Camera {
    pub position: Vec3,
    pub target: Vec3,
    pub up: Vec3,
    pub fov: f32,
    pub aspect: f32,
    pub near: f32,
    pub far: f32,
    // Orbit camera state
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

impl Camera {
    pub fn new(aspect: f32) -> Self {
        Self {
            position: Vec3::new(5.0, 5.0, 5.0),
            target: Vec3::ZERO,
            up: Vec3::Z,
            fov: 60.0_f32.to_radians(),
            aspect,
            near: 0.1,
            far: 1000.0,
            yaw: 45.0_f32.to_radians(),
            pitch: 30.0_f32.to_radians(),
            distance: 10.0,
        }
    }

    pub fn update_aspect(&mut self, aspect: f32) {
        self.aspect = aspect;
    }

    pub fn orbit(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.yaw += delta_yaw;
        self.pitch =
            (self.pitch + delta_pitch).clamp(-89.0_f32.to_radians(), 89.0_f32.to_radians());
        self.update_position_from_orbit();
    }

    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let forward = (self.target - self.position).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward).normalize();

        let scale = self.distance * 0.002;
        self.target += right * (-delta_x * scale) + up * (delta_y * scale);
        self.update_position_from_orbit();
    }

    pub fn zoom(&mut self, delta: f32) {
        self.distance = (self.distance * (1.0 - delta * 0.1)).clamp(0.1, 500.0);
        self.update_position_from_orbit();
    }

    fn update_position_from_orbit(&mut self) {
        let x = self.distance * self.pitch.cos() * self.yaw.cos();
        let y = self.distance * self.pitch.cos() * self.yaw.sin();
        let z = self.distance * self.pitch.sin();
        self.position = self.target + Vec3::new(x, y, z);
    }

    pub fn fit_all(&mut self, center: Vec3, radius: f32) {
        self.target = center;
        self.distance = radius * 2.5;
        self.update_position_from_orbit();
    }

    pub fn view_matrix(&self) -> Mat4 {
        Mat4::look_at_rh(self.position, self.target, self.up)
    }

    pub fn projection_matrix(&self) -> Mat4 {
        Mat4::perspective_rh(self.fov, self.aspect, self.near, self.far)
    }

    pub fn uniform(&self) -> CameraUniform {
        let view = self.view_matrix();
        let proj = self.projection_matrix();
        let view_proj = proj * view;

        CameraUniform {
            view_proj: view_proj.to_cols_array_2d(),
            view: view.to_cols_array_2d(),
            proj: proj.to_cols_array_2d(),
            eye: [self.position.x, self.position.y, self.position.z, 1.0],
        }
    }
}
