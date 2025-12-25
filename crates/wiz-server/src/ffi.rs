//! FFI bindings to C++ ROS2 bridge via cxx
//!
//! This module provides type-safe bindings to the C++ ROS2 bridge implementation.
//! When the `ros2` feature is enabled, it links against rclcpp for actual ROS2 communication.
//! Without the feature, it uses a mock implementation for development and testing.

#[allow(dead_code, clippy::module_inception)]
#[cxx::bridge(namespace = "wiz")]
pub mod ffi {
    /// Topic information returned from ROS2
    #[derive(Debug, Clone)]
    pub struct TopicInfoFfi {
        pub name: String,
        pub msg_type: String,
    }

    /// 3D Transform data
    #[derive(Debug, Clone)]
    pub struct TransformFfi {
        pub tx: f64,
        pub ty: f64,
        pub tz: f64,
        pub qx: f64,
        pub qy: f64,
        pub qz: f64,
        pub qw: f64,
    }

    /// PointCloud2 data header
    #[derive(Debug, Clone)]
    pub struct PointCloud2Header {
        pub timestamp_sec: i32,
        pub timestamp_nanosec: u32,
        pub frame_id: String,
        pub width: u32,
        pub height: u32,
        pub point_step: u32,
        pub row_step: u32,
        pub is_dense: bool,
    }

    /// LaserScan data
    #[derive(Debug, Clone)]
    pub struct LaserScanData {
        pub timestamp_sec: i32,
        pub timestamp_nanosec: u32,
        pub frame_id: String,
        pub angle_min: f32,
        pub angle_max: f32,
        pub angle_increment: f32,
        pub time_increment: f32,
        pub scan_time: f32,
        pub range_min: f32,
        pub range_max: f32,
    }

    unsafe extern "C++" {
        include!("wiz-server/cxx/bridge.hpp");

        type Ros2Bridge;

        /// Create a new ROS2 bridge instance
        fn create_bridge(node_name: &str, namespace_: &str) -> UniquePtr<Ros2Bridge>;

        /// Check if the bridge is initialized and spinning
        fn is_initialized(self: &Ros2Bridge) -> bool;

        /// Spin once to process callbacks
        fn spin_once(self: &Ros2Bridge);

        /// Get list of available topics
        fn list_topics(self: &Ros2Bridge) -> Vec<TopicInfoFfi>;

        /// Subscribe to a PointCloud2 topic
        fn subscribe_pointcloud2(self: &Ros2Bridge, topic: &str) -> bool;

        /// Subscribe to a LaserScan topic
        fn subscribe_laserscan(self: &Ros2Bridge, topic: &str) -> bool;

        /// Unsubscribe from a topic
        fn unsubscribe(self: &Ros2Bridge, topic: &str) -> bool;

        /// Get pending PointCloud2 data (returns empty if none available)
        fn get_pointcloud2_data(self: &Ros2Bridge, topic: &str) -> Vec<u8>;

        /// Get pending PointCloud2 header
        fn get_pointcloud2_header(self: &Ros2Bridge, topic: &str) -> PointCloud2Header;

        /// Get pending LaserScan data (ranges)
        fn get_laserscan_ranges(self: &Ros2Bridge, topic: &str) -> Vec<f32>;

        /// Get pending LaserScan intensities
        fn get_laserscan_intensities(self: &Ros2Bridge, topic: &str) -> Vec<f32>;

        /// Get pending LaserScan metadata
        fn get_laserscan_data(self: &Ros2Bridge, topic: &str) -> LaserScanData;

        /// Check if new data is available for a topic
        fn has_new_data(self: &Ros2Bridge, topic: &str) -> bool;

        /// Lookup transform between two frames
        fn lookup_transform(
            self: &Ros2Bridge,
            target_frame: &str,
            source_frame: &str,
        ) -> TransformFfi;

        /// Check if transform lookup succeeded (qw != 0 for valid transform)
        fn transform_valid(transform: &TransformFfi) -> bool;

        /// Get list of known TF frames
        fn get_tf_frames(self: &Ros2Bridge) -> Vec<String>;
    }
}

// Re-export types for convenience
#[allow(unused_imports)]
pub use ffi::{LaserScanData, PointCloud2Header, TopicInfoFfi, TransformFfi};
