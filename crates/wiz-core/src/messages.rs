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

// ============================================================================
// rosgraph_msgs/msg/Clock
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Clock {
    pub clock: f64, // seconds since epoch
}

// ============================================================================
// std_msgs/msg/String
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StringMsg {
    pub data: String,
}

// ============================================================================
// rcl_interfaces/msg/Log
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum LogLevel {
    Debug = 10,
    Info = 20,
    Warn = 30,
    Error = 40,
    Fatal = 50,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Log {
    pub stamp: f64,
    pub level: LogLevel,
    pub name: String,
    pub msg: String,
    pub file: String,
    pub function: String,
    pub line: u32,
}

// ============================================================================
// tf2_msgs/msg/TFMessage
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Transform {
    pub translation: [f64; 3],
    pub rotation: [f64; 4], // quaternion (x, y, z, w)
}

impl Default for Transform {
    fn default() -> Self {
        Self {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

// ============================================================================
// nav_msgs/msg/Odometry
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Twist {
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}

impl Default for Twist {
    fn default() -> Self {
        Self {
            linear: [0.0, 0.0, 0.0],
            angular: [0.0, 0.0, 0.0],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TwistStamped {
    pub header: Header,
    pub twist: Twist,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    pub covariance: Vec<f64>, // 36 elements (6x6 matrix)
}

impl Default for TwistWithCovariance {
    fn default() -> Self {
        Self {
            twist: Twist::default(),
            covariance: vec![0.0; 36],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    pub covariance: Vec<f64>, // 36 elements (6x6 matrix)
}

impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            covariance: vec![0.0; 36],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}

// ============================================================================
// sensor_msgs/msg/Imu
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Imu {
    pub header: Header,
    pub orientation: [f64; 4], // quaternion (x, y, z, w)
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: [f64; 3],
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: [f64; 3],
    pub linear_acceleration_covariance: [f64; 9],
}

// ============================================================================
// sensor_msgs/msg/JointState
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointState {
    pub header: Header,
    pub name: Vec<String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}

// ============================================================================
// sensor_msgs/msg/Image
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Image {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub encoding: String,
    pub is_bigendian: bool,
    pub step: u32,
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
}

// ============================================================================
// sensor_msgs/msg/CameraInfo
// ============================================================================

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RegionOfInterest {
    pub x_offset: u32,
    pub y_offset: u32,
    pub height: u32,
    pub width: u32,
    pub do_rectify: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraInfo {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub distortion_model: String,
    pub d: Vec<f64>,  // distortion parameters
    pub k: [f64; 9],  // intrinsic camera matrix
    pub r: [f64; 9],  // rectification matrix
    pub p: [f64; 12], // projection matrix
    pub binning_x: u32,
    pub binning_y: u32,
    pub roi: RegionOfInterest,
}
