//! Mock data generator for development and testing
//!
//! This module generates realistic mock data for various ROS2 message types
//! to allow testing the visualization without a real ROS2 environment.

use std::f32::consts::PI;
use wiz_core::{
    CameraInfo, Clock, Color, Header, Imu, JointState, LaserScan, Log, LogLevel, Marker,
    MarkerAction, MarkerArray, MarkerType, Odometry, PointCloud2, PointField, PointFieldType, Pose,
    PoseStamped, PoseWithCovariance, RegionOfInterest, StringMsg, TFMessage, Transform,
    TransformStamped, Twist, TwistStamped, TwistWithCovariance, Vector3,
};

/// Mock data generator with time-varying data
pub struct MockDataGenerator {
    tick: u64,
}

impl MockDataGenerator {
    pub fn new() -> Self {
        Self { tick: 0 }
    }

    /// Advance time and update internal state
    pub fn tick(&mut self) {
        self.tick = self.tick.wrapping_add(1);
    }

    /// Get current timestamp as f64 seconds
    fn timestamp(&self) -> f64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0)
    }

    /// Generate a mock LaserScan with time-varying obstacles
    pub fn generate_laser_scan(&self, _topic: &str) -> LaserScan {
        let num_rays = 360;
        let angle_min = 0.0_f32;
        let angle_max = 2.0 * PI;
        let angle_increment = (angle_max - angle_min) / num_rays as f32;

        let time_factor = self.tick as f32 * 0.05;

        let ranges: Vec<f32> = (0..num_rays)
            .map(|i| {
                let angle = angle_min + i as f32 * angle_increment;
                // Create moving obstacles with sinusoidal pattern
                let base_range = 5.0 + 2.0 * (angle * 3.0 + time_factor).sin();
                let obstacle1 = if (angle - time_factor * 0.2).abs() % (2.0 * PI) < 0.3 {
                    2.0
                } else {
                    f32::INFINITY
                };
                let obstacle2 = if ((angle - PI) - time_factor * 0.15).abs() % (2.0 * PI) < 0.2 {
                    3.5
                } else {
                    f32::INFINITY
                };
                base_range.min(obstacle1).min(obstacle2).clamp(0.1, 30.0)
            })
            .collect();

        let intensities: Vec<f32> = ranges.iter().map(|r| 1.0 - (r / 30.0)).collect();

        LaserScan {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "laser_frame".to_string(),
            },
            angle_min,
            angle_max,
            angle_increment,
            time_increment: 0.0,
            scan_time: 0.1,
            range_min: 0.1,
            range_max: 30.0,
            ranges,
            intensities,
        }
    }

    /// Generate a mock PointCloud2 with time-varying point positions
    pub fn generate_point_cloud(&self, _topic: &str) -> PointCloud2 {
        let num_points = 5000;
        let point_step = 20_u32; // x, y, z (12 bytes) + rgba (4 bytes) + padding (4 bytes)
        let time_factor = self.tick as f32 * 0.02;

        let mut data = Vec::with_capacity(num_points * point_step as usize);

        for i in 0..num_points {
            let t = i as f32 / num_points as f32;
            let angle = t * 2.0 * PI * 10.0 + time_factor;
            let radius = 3.0 + 2.0 * (t * 4.0 * PI + time_factor * 0.5).sin();

            // Spiral pattern
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            let z = t * 3.0 + 0.5 * (angle * 2.0).sin();

            // Write x, y, z as f32
            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
            data.extend_from_slice(&z.to_le_bytes());

            // RGBA color based on height and position
            let r = ((z / 3.5 * 255.0).clamp(0.0, 255.0)) as u8;
            let g = ((1.0 - t) * 200.0) as u8;
            let b = ((t * 255.0).clamp(0.0, 255.0)) as u8;
            let a = 255_u8;
            data.extend_from_slice(&[r, g, b, a]);

            // Padding to point_step
            data.extend_from_slice(&[0u8; 4]);
        }

        PointCloud2 {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "base_link".to_string(),
            },
            height: 1,
            width: num_points as u32,
            fields: vec![
                PointField {
                    name: "x".to_string(),
                    offset: 0,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                PointField {
                    name: "y".to_string(),
                    offset: 4,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                PointField {
                    name: "z".to_string(),
                    offset: 8,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                PointField {
                    name: "rgba".to_string(),
                    offset: 12,
                    datatype: PointFieldType::Uint32,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step,
            row_step: point_step * num_points as u32,
            data,
            is_dense: true,
        }
    }

    /// Generate a simpler ground plane point cloud
    pub fn generate_ground_plane(&self) -> PointCloud2 {
        let grid_size = 50;
        let spacing = 0.5_f32;
        let num_points = grid_size * grid_size;
        let point_step = 20_u32;
        let time_factor = self.tick as f32 * 0.03;

        let mut data = Vec::with_capacity(num_points * point_step as usize);

        for i in 0..grid_size {
            for j in 0..grid_size {
                let x = (i as f32 - grid_size as f32 / 2.0) * spacing;
                let y = (j as f32 - grid_size as f32 / 2.0) * spacing;
                let z = 0.1 * ((x * 0.5 + time_factor).sin() + (y * 0.5).cos());

                data.extend_from_slice(&x.to_le_bytes());
                data.extend_from_slice(&y.to_le_bytes());
                data.extend_from_slice(&z.to_le_bytes());

                // Checkerboard color pattern
                let is_white = (i + j) % 2 == 0;
                let (r, g, b) = if is_white {
                    (200u8, 200u8, 200u8)
                } else {
                    (100u8, 100u8, 100u8)
                };
                data.extend_from_slice(&[r, g, b, 255u8]);
                data.extend_from_slice(&[0u8; 4]);
            }
        }

        PointCloud2 {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            height: 1,
            width: num_points as u32,
            fields: vec![
                PointField {
                    name: "x".to_string(),
                    offset: 0,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                PointField {
                    name: "y".to_string(),
                    offset: 4,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                PointField {
                    name: "z".to_string(),
                    offset: 8,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                PointField {
                    name: "rgba".to_string(),
                    offset: 12,
                    datatype: PointFieldType::Uint32,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step,
            row_step: point_step * num_points as u32,
            data,
            is_dense: true,
        }
    }

    /// Generate a mock PoseStamped representing a moving robot
    pub fn generate_pose(&self, _topic: &str) -> PoseStamped {
        let time_factor = self.tick as f32 * 0.02;

        // Robot moves in a figure-8 pattern
        let t = time_factor * 0.5;
        let x = 3.0 * (t).sin();
        let y = 1.5 * (2.0 * t).sin();
        let z = 0.0;

        // Orientation follows the movement direction
        let dx = 3.0 * t.cos();
        let dy = 3.0 * (2.0 * t).cos();
        let yaw = dy.atan2(dx);

        // Convert yaw to quaternion (rotation around Z axis)
        let half_yaw = yaw / 2.0;
        let qx = 0.0;
        let qy = 0.0;
        let qz = half_yaw.sin();
        let qw = half_yaw.cos();

        PoseStamped {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            pose: Pose {
                position: [x as f64, y as f64, z],
                orientation: [qx, qy, qz as f64, qw as f64],
            },
        }
    }

    /// Generate a mock MarkerArray with various marker types
    pub fn generate_marker_array(&self, _topic: &str) -> MarkerArray {
        let time_factor = self.tick as f32 * 0.02;
        let mut markers = Vec::new();

        // Cube marker - stationary
        markers.push(Marker {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            ns: "demo".to_string(),
            id: 0,
            marker_type: MarkerType::Cube,
            action: MarkerAction::Add,
            pose: Pose {
                position: [2.0, 0.0, 0.5],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
            scale: Vector3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            color: Color {
                r: 0.0,
                g: 0.8,
                b: 0.2,
                a: 0.8,
            },
            lifetime: 0.0,
            frame_locked: false,
            points: Vec::new(),
            colors: Vec::new(),
            text: String::new(),
            mesh_resource: String::new(),
        });

        // Sphere marker - bouncing
        let bounce_z = 1.0 + 0.5 * (time_factor * 2.0).sin().abs();
        markers.push(Marker {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            ns: "demo".to_string(),
            id: 1,
            marker_type: MarkerType::Sphere,
            action: MarkerAction::Add,
            pose: Pose {
                position: [-2.0, 0.0, bounce_z as f64],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
            scale: Vector3 {
                x: 0.8,
                y: 0.8,
                z: 0.8,
            },
            color: Color {
                r: 0.8,
                g: 0.2,
                b: 0.8,
                a: 0.9,
            },
            lifetime: 0.0,
            frame_locked: false,
            points: Vec::new(),
            colors: Vec::new(),
            text: String::new(),
            mesh_resource: String::new(),
        });

        // Cylinder marker - rotating
        let rot_angle = time_factor * 0.5;
        let qz = (rot_angle / 2.0).sin();
        let qw = (rot_angle / 2.0).cos();
        markers.push(Marker {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            ns: "demo".to_string(),
            id: 2,
            marker_type: MarkerType::Cylinder,
            action: MarkerAction::Add,
            pose: Pose {
                position: [0.0, 2.0, 0.75],
                orientation: [0.0, 0.0, qz as f64, qw as f64],
            },
            scale: Vector3 {
                x: 0.5,
                y: 0.5,
                z: 1.5,
            },
            color: Color {
                r: 0.2,
                g: 0.6,
                b: 0.9,
                a: 0.85,
            },
            lifetime: 0.0,
            frame_locked: false,
            points: Vec::new(),
            colors: Vec::new(),
            text: String::new(),
            mesh_resource: String::new(),
        });

        // Arrow marker - pointing in a rotating direction
        let arrow_angle = time_factor * 0.3;
        let arrow_qz = (arrow_angle / 2.0).sin();
        let arrow_qw = (arrow_angle / 2.0).cos();
        markers.push(Marker {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            ns: "demo".to_string(),
            id: 3,
            marker_type: MarkerType::Arrow,
            action: MarkerAction::Add,
            pose: Pose {
                position: [0.0, -2.0, 0.5],
                orientation: [0.0, 0.0, arrow_qz as f64, arrow_qw as f64],
            },
            scale: Vector3 {
                x: 1.5,
                y: 0.2,
                z: 0.2,
            },
            color: Color {
                r: 1.0,
                g: 0.5,
                b: 0.0,
                a: 1.0,
            },
            lifetime: 0.0,
            frame_locked: false,
            points: Vec::new(),
            colors: Vec::new(),
            text: String::new(),
            mesh_resource: String::new(),
        });

        // LineStrip marker - waving line
        let mut line_points = Vec::new();
        for i in 0..20 {
            let x = -3.0 + i as f64 * 0.3;
            let y = 4.0 + 0.5 * ((x + time_factor as f64) * 2.0).sin();
            let z = 0.2;
            line_points.push([x, y, z]);
        }
        markers.push(Marker {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            ns: "demo".to_string(),
            id: 4,
            marker_type: MarkerType::LineStrip,
            action: MarkerAction::Add,
            pose: Pose::default(),
            scale: Vector3 {
                x: 0.05, // Line width
                y: 0.05,
                z: 0.05,
            },
            color: Color {
                r: 1.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            },
            lifetime: 0.0,
            frame_locked: false,
            points: line_points,
            colors: Vec::new(),
            text: String::new(),
            mesh_resource: String::new(),
        });

        // SphereList marker - orbiting spheres
        let mut sphere_points = Vec::new();
        let mut sphere_colors = Vec::new();
        for i in 0..8 {
            let angle = (i as f32 / 8.0) * 2.0 * PI + time_factor * 0.5;
            let radius = 1.5;
            let x = -4.0 + radius * angle.cos();
            let y = -4.0 + radius * angle.sin();
            let z = 0.3;
            sphere_points.push([x as f64, y as f64, z]);

            // Color gradient around the circle
            let hue = i as f32 / 8.0;
            sphere_colors.push(Color {
                r: (hue * 2.0 * PI).sin() * 0.5 + 0.5,
                g: ((hue + 0.33) * 2.0 * PI).sin() * 0.5 + 0.5,
                b: ((hue + 0.66) * 2.0 * PI).sin() * 0.5 + 0.5,
                a: 0.9,
            });
        }
        markers.push(Marker {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            ns: "demo".to_string(),
            id: 5,
            marker_type: MarkerType::SphereList,
            action: MarkerAction::Add,
            pose: Pose::default(),
            scale: Vector3 {
                x: 0.3,
                y: 0.3,
                z: 0.3,
            },
            color: Color::default(), // Will use per-point colors
            lifetime: 0.0,
            frame_locked: false,
            points: sphere_points,
            colors: sphere_colors,
            text: String::new(),
            mesh_resource: String::new(),
        });

        // CubeList marker - stacked cubes
        let mut cube_points = Vec::new();
        let mut cube_colors = Vec::new();
        for i in 0..5 {
            let x = 4.0;
            let y = -2.0;
            let z = 0.25 + i as f64 * 0.5;
            cube_points.push([x, y, z]);

            // Gradient from blue to red
            let t = i as f32 / 4.0;
            cube_colors.push(Color {
                r: t,
                g: 0.2,
                b: 1.0 - t,
                a: 0.85,
            });
        }
        markers.push(Marker {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            ns: "demo".to_string(),
            id: 6,
            marker_type: MarkerType::CubeList,
            action: MarkerAction::Add,
            pose: Pose::default(),
            scale: Vector3 {
                x: 0.4,
                y: 0.4,
                z: 0.4,
            },
            color: Color::default(),
            lifetime: 0.0,
            frame_locked: false,
            points: cube_points,
            colors: cube_colors,
            text: String::new(),
            mesh_resource: String::new(),
        });

        MarkerArray { markers }
    }

    /// Generate a mock TFMessage with multiple transforms
    pub fn generate_tf_message(&self, _topic: &str) -> TFMessage {
        let time_factor = self.tick as f32 * 0.02;

        let mut transforms = Vec::new();

        // map -> odom transform (slowly drifting)
        let drift_x = 0.1 * (time_factor * 0.1).sin();
        let drift_y = 0.05 * (time_factor * 0.15).cos();
        transforms.push(TransformStamped {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "map".to_string(),
            },
            child_frame_id: "odom".to_string(),
            transform: Transform {
                translation: [drift_x as f64, drift_y as f64, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        });

        // odom -> base_link transform (robot moving in figure-8)
        let t = time_factor * 0.5;
        let x = 3.0 * t.sin();
        let y = 1.5 * (2.0 * t).sin();
        let yaw = (3.0 * t.cos()).atan2(3.0 * (2.0 * t).cos());
        let half_yaw = yaw / 2.0;
        transforms.push(TransformStamped {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "odom".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            transform: Transform {
                translation: [x as f64, y as f64, 0.0],
                rotation: [0.0, 0.0, half_yaw.sin() as f64, half_yaw.cos() as f64],
            },
        });

        // base_link -> laser_frame transform (fixed)
        transforms.push(TransformStamped {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "base_link".to_string(),
            },
            child_frame_id: "laser_frame".to_string(),
            transform: Transform {
                translation: [0.1, 0.0, 0.2],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        });

        // base_link -> imu_link transform (fixed)
        transforms.push(TransformStamped {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "base_link".to_string(),
            },
            child_frame_id: "imu_link".to_string(),
            transform: Transform {
                translation: [0.0, 0.0, 0.1],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        });

        // base_link -> camera_link transform (fixed)
        transforms.push(TransformStamped {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "base_link".to_string(),
            },
            child_frame_id: "camera_link".to_string(),
            transform: Transform {
                translation: [0.15, 0.0, 0.3],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        });

        TFMessage { transforms }
    }

    /// Generate a mock Odometry message
    pub fn generate_odometry(&self, _topic: &str) -> Odometry {
        let time_factor = self.tick as f32 * 0.02;

        // Robot moves in a figure-8 pattern
        let t = time_factor * 0.5;
        let x = 3.0 * t.sin();
        let y = 1.5 * (2.0 * t).sin();
        let z = 0.0;

        // Velocity
        let vx = 3.0 * 0.5 * t.cos();
        let vy = 1.5 * 2.0 * 0.5 * (2.0 * t).cos();

        // Orientation follows the movement direction
        let yaw = vy.atan2(vx);
        let half_yaw = yaw / 2.0;

        Odometry {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "odom".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            pose: PoseWithCovariance {
                pose: Pose {
                    position: [x as f64, y as f64, z],
                    orientation: [0.0, 0.0, half_yaw.sin() as f64, half_yaw.cos() as f64],
                },
                covariance: vec![0.0; 36],
            },
            twist: TwistWithCovariance {
                twist: Twist {
                    linear: [vx as f64, vy as f64, 0.0],
                    angular: [0.0, 0.0, 0.5], // constant angular velocity
                },
                covariance: vec![0.0; 36],
            },
        }
    }

    /// Generate a mock Imu message
    pub fn generate_imu(&self, _topic: &str) -> Imu {
        let time_factor = self.tick as f32 * 0.02;

        // Simulate some tilting motion
        let roll = 0.1 * (time_factor * 2.0).sin();
        let pitch = 0.05 * (time_factor * 1.5).cos();
        let yaw = time_factor * 0.3;

        // Convert euler to quaternion (simplified)
        let (sr, cr) = (roll / 2.0).sin_cos();
        let (sp, cp) = (pitch / 2.0).sin_cos();
        let (sy, cy) = (yaw / 2.0).sin_cos();

        let qx = sr * cp * cy - cr * sp * sy;
        let qy = cr * sp * cy + sr * cp * sy;
        let qz = cr * cp * sy - sr * sp * cy;
        let qw = cr * cp * cy + sr * sp * sy;

        // Simulate accelerometer (gravity + noise)
        let ax = 0.1 * (time_factor * 5.0).sin();
        let ay = 0.1 * (time_factor * 4.0).cos();
        let az = 9.81 + 0.05 * (time_factor * 3.0).sin();

        // Simulate gyroscope
        let gx = 2.0 * 0.1 * (time_factor * 2.0).cos();
        let gy = -1.5 * 0.05 * (time_factor * 1.5).sin();
        let gz: f32 = 0.3;

        Imu {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "imu_link".to_string(),
            },
            orientation: [qx as f64, qy as f64, qz as f64, qw as f64],
            orientation_covariance: [0.0; 9],
            angular_velocity: [gx as f64, gy as f64, gz as f64],
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration: [ax as f64, ay as f64, az as f64],
            linear_acceleration_covariance: [0.0; 9],
        }
    }

    /// Generate a mock Twist message (velocity command)
    pub fn generate_twist(&self, _topic: &str) -> TwistStamped {
        let time_factor = self.tick as f32 * 0.02;

        // Simulate varying velocity commands
        let linear_x = 0.5 * (1.0 + (time_factor * 0.5).sin());
        let angular_z = 0.3 * (time_factor * 0.3).sin();

        TwistStamped {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "base_link".to_string(),
            },
            twist: Twist {
                linear: [linear_x as f64, 0.0, 0.0],
                angular: [0.0, 0.0, angular_z as f64],
            },
        }
    }

    /// Generate a mock JointState message
    pub fn generate_joint_state(&self, _topic: &str) -> JointState {
        let time_factor = self.tick as f32 * 0.02;

        // Simulate a robot with several joints
        let joint_names = vec![
            "wheel_left_joint".to_string(),
            "wheel_right_joint".to_string(),
            "head_pan_joint".to_string(),
            "head_tilt_joint".to_string(),
        ];

        // Wheel joints rotate continuously
        let wheel_pos = time_factor * 2.0;
        let wheel_vel: f32 = 2.0;

        // Head joints oscillate
        let head_pan = 0.5 * (time_factor * 0.5).sin();
        let head_tilt = 0.3 * (time_factor * 0.3).cos();

        JointState {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "".to_string(),
            },
            name: joint_names,
            position: vec![
                wheel_pos as f64,
                wheel_pos as f64,
                head_pan as f64,
                head_tilt as f64,
            ],
            velocity: vec![wheel_vel as f64, wheel_vel as f64, 0.25, 0.15],
            effort: vec![1.0, 1.0, 0.1, 0.1],
        }
    }

    /// Generate a mock CameraInfo message
    pub fn generate_camera_info(&self, _topic: &str) -> CameraInfo {
        CameraInfo {
            header: Header {
                stamp: self.timestamp(),
                frame_id: "camera_link".to_string(),
            },
            height: 480,
            width: 640,
            distortion_model: "plumb_bob".to_string(),
            d: vec![0.0, 0.0, 0.0, 0.0, 0.0],
            k: [
                525.0, 0.0, 320.0, // fx, 0, cx
                0.0, 525.0, 240.0, // 0, fy, cy
                0.0, 0.0, 1.0, // 0, 0, 1
            ],
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p: [
                525.0, 0.0, 320.0, 0.0, // fx, 0, cx, 0
                0.0, 525.0, 240.0, 0.0, // 0, fy, cy, 0
                0.0, 0.0, 1.0, 0.0, // 0, 0, 1, 0
            ],
            binning_x: 1,
            binning_y: 1,
            roi: RegionOfInterest::default(),
        }
    }

    /// Generate a mock Clock message
    pub fn generate_clock(&self, _topic: &str) -> Clock {
        Clock {
            clock: self.timestamp(),
        }
    }

    /// Generate a mock String message
    pub fn generate_string(&self, _topic: &str) -> StringMsg {
        let messages = [
            "Robot is operational",
            "Navigation active",
            "Sensors nominal",
            "Battery level: 85%",
            "System status: OK",
        ];
        let idx = (self.tick as usize) % messages.len();
        StringMsg {
            data: messages[idx].to_string(),
        }
    }

    /// Generate a mock Log message
    pub fn generate_log(&self, _topic: &str) -> Log {
        let log_entries = [
            (LogLevel::Info, "navigation", "Path planning complete"),
            (LogLevel::Debug, "sensor_fusion", "IMU data received"),
            (LogLevel::Warn, "battery_monitor", "Battery below 30%"),
            (LogLevel::Info, "motor_controller", "Motors initialized"),
            (LogLevel::Debug, "localization", "Pose updated"),
        ];
        let idx = (self.tick as usize) % log_entries.len();
        let (level, name, msg) = log_entries[idx];

        Log {
            stamp: self.timestamp(),
            level,
            name: name.to_string(),
            msg: msg.to_string(),
            file: "mock_node.cpp".to_string(),
            function: "update".to_string(),
            line: 42 + (self.tick % 100) as u32,
        }
    }
}

impl Default for MockDataGenerator {
    fn default() -> Self {
        Self::new()
    }
}
