use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub stamp: f64,
    pub frame_id: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PointFieldType {
    Int8 = 1,
    Uint8 = 2,
    Int16 = 3,
    Uint16 = 4,
    Int32 = 5,
    Uint32 = 6,
    Float32 = 7,
    Float64 = 8,
}

impl PointFieldType {
    pub fn size(&self) -> usize {
        match self {
            Self::Int8 | Self::Uint8 => 1,
            Self::Int16 | Self::Uint16 => 2,
            Self::Int32 | Self::Uint32 | Self::Float32 => 4,
            Self::Float64 => 8,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: PointFieldType,
    pub count: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
    pub is_dense: bool,
}

impl PointCloud2 {
    pub fn point_count(&self) -> usize {
        (self.height * self.width) as usize
    }

    pub fn find_field(&self, name: &str) -> Option<&PointField> {
        self.fields.iter().find(|f| f.name == name)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
}

impl LaserScan {
    pub fn to_points(&self) -> Vec<[f32; 3]> {
        let mut points = Vec::with_capacity(self.ranges.len());
        let mut angle = self.angle_min;

        for &range in &self.ranges {
            if range >= self.range_min && range <= self.range_max {
                let x = range * angle.cos();
                let y = range * angle.sin();
                points.push([x, y, 0.0]);
            }
            angle += self.angle_increment;
        }
        points
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub position: [f64; 3],
    pub orientation: [f64; 4], // quaternion (x, y, z, w)
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Path {
    pub header: Header,
    pub poses: Vec<PoseStamped>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum MarkerType {
    Arrow = 0,
    Cube = 1,
    Sphere = 2,
    Cylinder = 3,
    LineStrip = 4,
    LineList = 5,
    CubeList = 6,
    SphereList = 7,
    Points = 8,
    TextViewFacing = 9,
    MeshResource = 10,
    TriangleList = 11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum MarkerAction {
    Add = 0,
    Modify = 1,
    Delete = 2,
    DeleteAll = 3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Default for Color {
    fn default() -> Self {
        Self {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Vector3 {
    fn default() -> Self {
        Self {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Marker {
    pub header: Header,
    pub ns: String,
    pub id: i32,
    pub marker_type: MarkerType,
    pub action: MarkerAction,
    pub pose: Pose,
    pub scale: Vector3,
    pub color: Color,
    pub lifetime: f64,
    pub frame_locked: bool,
    pub points: Vec<[f64; 3]>,
    pub colors: Vec<Color>,
    pub text: String,
    pub mesh_resource: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarkerArray {
    pub markers: Vec<Marker>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicInfo {
    pub name: String,
    pub msg_type: String,
}
