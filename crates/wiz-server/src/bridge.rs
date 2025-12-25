//! ROS2 Bridge module
//!
//! This module provides a Rust wrapper around the C++ ROS2 bridge.
//! It uses cxx for type-safe FFI bindings to rclcpp.

use std::sync::Arc;

use parking_lot::Mutex;
use wiz_core::{LaserScan, PointCloud2, PointFieldType, TopicInfo, Transform3D};

use crate::ffi;

/// ROS2 Bridge wrapper
///
/// Provides a safe Rust interface to the C++ ROS2 bridge.
/// Thread-safe via internal mutex.
#[allow(dead_code, clippy::arc_with_non_send_sync)]
pub struct Ros2Bridge {
    inner: Arc<Mutex<cxx::UniquePtr<ffi::ffi::Ros2Bridge>>>,
}

#[allow(dead_code, clippy::arc_with_non_send_sync)]
impl Ros2Bridge {
    /// Create a new ROS2 bridge instance
    pub fn new(node_name: &str, namespace: &str) -> Self {
        let bridge = ffi::ffi::create_bridge(node_name, namespace);
        Self {
            inner: Arc::new(Mutex::new(bridge)),
        }
    }

    /// Check if the bridge is initialized
    pub fn is_initialized(&self) -> bool {
        let guard = self.inner.lock();
        guard.is_initialized()
    }

    /// Spin once to process callbacks
    pub fn spin_once(&self) {
        let guard = self.inner.lock();
        guard.spin_once();
    }

    /// Get list of available topics
    pub fn list_topics(&self) -> Vec<TopicInfo> {
        let guard = self.inner.lock();
        guard
            .list_topics()
            .into_iter()
            .map(|t| TopicInfo {
                name: t.name.to_string(),
                msg_type: t.msg_type.to_string(),
            })
            .collect()
    }

    /// Subscribe to a PointCloud2 topic
    pub fn subscribe_pointcloud2(&self, topic: &str) -> bool {
        let guard = self.inner.lock();
        guard.subscribe_pointcloud2(topic)
    }

    /// Subscribe to a LaserScan topic
    pub fn subscribe_laserscan(&self, topic: &str) -> bool {
        let guard = self.inner.lock();
        guard.subscribe_laserscan(topic)
    }

    /// Unsubscribe from a topic
    pub fn unsubscribe(&self, topic: &str) -> bool {
        let guard = self.inner.lock();
        guard.unsubscribe(topic)
    }

    /// Check if new data is available for a topic
    pub fn has_new_data(&self, topic: &str) -> bool {
        let guard = self.inner.lock();
        guard.has_new_data(topic)
    }

    /// Get PointCloud2 data from a topic
    pub fn get_pointcloud2(&self, topic: &str) -> Option<PointCloud2> {
        let guard = self.inner.lock();

        if !guard.has_new_data(topic) {
            return None;
        }

        let header = guard.get_pointcloud2_header(topic);
        let data = guard.get_pointcloud2_data(topic);

        Some(PointCloud2 {
            header: wiz_core::Header {
                stamp: header.timestamp_sec as f64
                    + header.timestamp_nanosec as f64 / 1_000_000_000.0,
                frame_id: header.frame_id.to_string(),
            },
            width: header.width,
            height: header.height,
            fields: vec![
                wiz_core::PointField {
                    name: "x".to_string(),
                    offset: 0,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                wiz_core::PointField {
                    name: "y".to_string(),
                    offset: 4,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                wiz_core::PointField {
                    name: "z".to_string(),
                    offset: 8,
                    datatype: PointFieldType::Float32,
                    count: 1,
                },
                wiz_core::PointField {
                    name: "rgba".to_string(),
                    offset: 16,
                    datatype: PointFieldType::Uint32,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step: header.point_step,
            row_step: header.row_step,
            data: data.into_iter().collect(),
            is_dense: header.is_dense,
        })
    }

    /// Get LaserScan data from a topic
    pub fn get_laserscan(&self, topic: &str) -> Option<LaserScan> {
        let guard = self.inner.lock();

        if !guard.has_new_data(topic) {
            return None;
        }

        let meta = guard.get_laserscan_data(topic);
        let ranges = guard.get_laserscan_ranges(topic);
        let intensities = guard.get_laserscan_intensities(topic);

        Some(LaserScan {
            header: wiz_core::Header {
                stamp: meta.timestamp_sec as f64 + meta.timestamp_nanosec as f64 / 1_000_000_000.0,
                frame_id: meta.frame_id.to_string(),
            },
            angle_min: meta.angle_min,
            angle_max: meta.angle_max,
            angle_increment: meta.angle_increment,
            time_increment: meta.time_increment,
            scan_time: meta.scan_time,
            range_min: meta.range_min,
            range_max: meta.range_max,
            ranges: ranges.into_iter().collect(),
            intensities: intensities.into_iter().collect(),
        })
    }

    /// Lookup transform between two frames
    pub fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
    ) -> Result<Transform3D, String> {
        let guard = self.inner.lock();
        let tf = guard.lookup_transform(target_frame, source_frame);

        if ffi::ffi::transform_valid(&tf) {
            Ok(Transform3D {
                translation: [tf.tx, tf.ty, tf.tz],
                rotation: [tf.qx, tf.qy, tf.qz, tf.qw],
            })
        } else {
            Err(format!(
                "Transform from {} to {} not available",
                source_frame, target_frame
            ))
        }
    }

    /// Get list of known TF frames
    pub fn get_tf_frames(&self) -> Vec<String> {
        let guard = self.inner.lock();
        guard
            .get_tf_frames()
            .into_iter()
            .map(|s| s.to_string())
            .collect()
    }
}

// Make Ros2Bridge Send + Sync safe
unsafe impl Send for Ros2Bridge {}
unsafe impl Sync for Ros2Bridge {}

impl Clone for Ros2Bridge {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
        }
    }
}
