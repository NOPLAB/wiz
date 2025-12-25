//! Mock data generator for development and testing
//!
//! This module generates realistic mock data for various ROS2 message types
//! to allow testing the visualization without a real ROS2 environment.

use std::f32::consts::PI;
use wiz_core::{Header, LaserScan, PointCloud2, PointField, PointFieldType};

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
}

impl Default for MockDataGenerator {
    fn default() -> Self {
        Self::new()
    }
}
