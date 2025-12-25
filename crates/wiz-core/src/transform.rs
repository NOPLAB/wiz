use glam::{Mat4, Quat, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform3D {
    pub translation: [f64; 3],
    pub rotation: [f64; 4], // quaternion (x, y, z, w)
}

impl Transform3D {
    pub fn identity() -> Self {
        Self {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    pub fn to_mat4(&self) -> Mat4 {
        let translation = Vec3::new(
            self.translation[0] as f32,
            self.translation[1] as f32,
            self.translation[2] as f32,
        );
        let rotation = Quat::from_xyzw(
            self.rotation[0] as f32,
            self.rotation[1] as f32,
            self.rotation[2] as f32,
            self.rotation[3] as f32,
        );
        Mat4::from_rotation_translation(rotation, translation)
    }

    pub fn inverse(&self) -> Self {
        let q = Quat::from_xyzw(
            self.rotation[0] as f32,
            self.rotation[1] as f32,
            self.rotation[2] as f32,
            self.rotation[3] as f32,
        );
        let inv_q = q.inverse();
        let t = Vec3::new(
            self.translation[0] as f32,
            self.translation[1] as f32,
            self.translation[2] as f32,
        );
        let inv_t = inv_q * (-t);

        Self {
            translation: [inv_t.x as f64, inv_t.y as f64, inv_t.z as f64],
            rotation: [
                inv_q.x as f64,
                inv_q.y as f64,
                inv_q.z as f64,
                inv_q.w as f64,
            ],
        }
    }

    pub fn compose(&self, other: &Transform3D) -> Self {
        let q1 = Quat::from_xyzw(
            self.rotation[0] as f32,
            self.rotation[1] as f32,
            self.rotation[2] as f32,
            self.rotation[3] as f32,
        );
        let q2 = Quat::from_xyzw(
            other.rotation[0] as f32,
            other.rotation[1] as f32,
            other.rotation[2] as f32,
            other.rotation[3] as f32,
        );
        let t1 = Vec3::new(
            self.translation[0] as f32,
            self.translation[1] as f32,
            self.translation[2] as f32,
        );
        let t2 = Vec3::new(
            other.translation[0] as f32,
            other.translation[1] as f32,
            other.translation[2] as f32,
        );

        let new_q = q1 * q2;
        let new_t = q1 * t2 + t1;

        Self {
            translation: [new_t.x as f64, new_t.y as f64, new_t.z as f64],
            rotation: [
                new_q.x as f64,
                new_q.y as f64,
                new_q.z as f64,
                new_q.w as f64,
            ],
        }
    }
}

impl Default for Transform3D {
    fn default() -> Self {
        Self::identity()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TfFrame {
    pub frame_id: String,
    pub parent_id: Option<String>,
    pub transform: Transform3D,
    pub timestamp: f64,
}
