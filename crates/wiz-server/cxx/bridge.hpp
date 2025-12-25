#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "rust/cxx.h"

namespace wiz {

// Forward declarations for cxx-generated structs
// These are defined by cxx from the Rust side
struct TopicInfoFfi;
struct TransformFfi;
struct PointCloud2Header;
struct LaserScanData;

/// ROS2 Bridge class
/// Provides interface to ROS2 via rclcpp (when ros2 feature is enabled)
/// or mock implementation for development
///
/// All public methods are const to work with cxx's &self pattern.
/// Internal state is managed through mutable members.
class Ros2Bridge {
public:
    Ros2Bridge(rust::Str node_name, rust::Str namespace_);
    ~Ros2Bridge();

    // Lifecycle
    bool is_initialized() const;
    void spin_once() const;

    // Topic discovery
    rust::Vec<TopicInfoFfi> list_topics() const;

    // Subscriptions
    bool subscribe_pointcloud2(rust::Str topic) const;
    bool subscribe_laserscan(rust::Str topic) const;
    bool unsubscribe(rust::Str topic) const;

    // Data retrieval
    rust::Vec<uint8_t> get_pointcloud2_data(rust::Str topic) const;
    PointCloud2Header get_pointcloud2_header(rust::Str topic) const;
    rust::Vec<float> get_laserscan_ranges(rust::Str topic) const;
    rust::Vec<float> get_laserscan_intensities(rust::Str topic) const;
    LaserScanData get_laserscan_data(rust::Str topic) const;
    bool has_new_data(rust::Str topic) const;

    // TF
    TransformFfi lookup_transform(rust::Str target_frame, rust::Str source_frame) const;
    rust::Vec<rust::String> get_tf_frames() const;

private:
    struct Impl;
    // mutable to allow modification in const methods
    mutable std::unique_ptr<Impl> impl_;
};

/// Factory function
std::unique_ptr<Ros2Bridge> create_bridge(rust::Str node_name, rust::Str namespace_);

/// Check if transform is valid
bool transform_valid(const TransformFfi& transform);

} // namespace wiz
