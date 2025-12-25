// Include cxx-generated header first (contains struct definitions)
#include "wiz-server/src/ffi.rs.h"
#include "wiz-server/cxx/bridge.hpp"

#include <cmath>
#include <map>
#include <mutex>
#include <random>
#include <chrono>

namespace wiz {

// Implementation details
struct Ros2Bridge::Impl {
    std::string node_name;
    std::string namespace_;
    bool initialized = false;

    // Mock subscriptions
    mutable std::map<std::string, std::string> subscriptions; // topic -> msg_type

    // Mock data storage
    mutable std::mutex data_mutex;
    mutable std::map<std::string, bool> new_data_flags;

    // Mock TF frames
    std::vector<std::string> tf_frames = {"map", "odom", "base_link", "laser_frame"};

    // Random generator for mock data
    mutable std::mt19937 rng{std::random_device{}()};

    Impl(const std::string& name, const std::string& ns)
        : node_name(name), namespace_(ns) {
#ifdef WIZ_ROS2_ENABLED
        // TODO: Initialize rclcpp node
        // rclcpp::init(0, nullptr);
        // node_ = std::make_shared<rclcpp::Node>(name, ns);
#endif
        initialized = true;
    }

    ~Impl() {
#ifdef WIZ_ROS2_ENABLED
        // TODO: Shutdown rclcpp
        // rclcpp::shutdown();
#endif
    }
};

Ros2Bridge::Ros2Bridge(rust::Str node_name, rust::Str namespace_)
    : impl_(std::make_unique<Impl>(
          std::string(node_name.data(), node_name.size()),
          std::string(namespace_.data(), namespace_.size()))) {
}

Ros2Bridge::~Ros2Bridge() = default;

bool Ros2Bridge::is_initialized() const {
    return impl_ && impl_->initialized;
}

void Ros2Bridge::spin_once() const {
#ifdef WIZ_ROS2_ENABLED
    // TODO: rclcpp::spin_some(node_);
#else
    // Mock: Generate some fake data periodically
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    for (auto& [topic, _] : impl_->subscriptions) {
        impl_->new_data_flags[topic] = true;
    }
#endif
}

rust::Vec<TopicInfoFfi> Ros2Bridge::list_topics() const {
    rust::Vec<TopicInfoFfi> topics;

#ifdef WIZ_ROS2_ENABLED
    // TODO: Query actual topics from ROS2
#else
    // Mock topics for development
    topics.push_back(TopicInfoFfi{
        rust::String("/velodyne_points"),
        rust::String("sensor_msgs/msg/PointCloud2")
    });
    topics.push_back(TopicInfoFfi{
        rust::String("/scan"),
        rust::String("sensor_msgs/msg/LaserScan")
    });
    topics.push_back(TopicInfoFfi{
        rust::String("/tf"),
        rust::String("tf2_msgs/msg/TFMessage")
    });
    topics.push_back(TopicInfoFfi{
        rust::String("/odom"),
        rust::String("nav_msgs/msg/Odometry")
    });
#endif

    return topics;
}

bool Ros2Bridge::subscribe_pointcloud2(rust::Str topic) const {
    std::string topic_str(topic.data(), topic.size());

#ifdef WIZ_ROS2_ENABLED
    // TODO: Create actual ROS2 subscription
#endif

    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->subscriptions[topic_str] = "sensor_msgs/msg/PointCloud2";
    impl_->new_data_flags[topic_str] = false;
    return true;
}

bool Ros2Bridge::subscribe_laserscan(rust::Str topic) const {
    std::string topic_str(topic.data(), topic.size());

#ifdef WIZ_ROS2_ENABLED
    // TODO: Create actual ROS2 subscription
#endif

    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->subscriptions[topic_str] = "sensor_msgs/msg/LaserScan";
    impl_->new_data_flags[topic_str] = false;
    return true;
}

bool Ros2Bridge::unsubscribe(rust::Str topic) const {
    std::string topic_str(topic.data(), topic.size());

    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    auto it = impl_->subscriptions.find(topic_str);
    if (it != impl_->subscriptions.end()) {
        impl_->subscriptions.erase(it);
        impl_->new_data_flags.erase(topic_str);
        return true;
    }
    return false;
}

rust::Vec<uint8_t> Ros2Bridge::get_pointcloud2_data(rust::Str topic) const {
    rust::Vec<uint8_t> data;

#ifdef WIZ_ROS2_ENABLED
    // TODO: Return actual point cloud data
#else
    // Mock: Generate a simple point cloud (1000 points, XYZRGBA format)
    const size_t num_points = 1000;
    const size_t point_step = 32; // x,y,z,_,r,g,b,a (4 floats + 4 bytes)
    data.reserve(num_points * point_step);

    std::uniform_real_distribution<float> dist(-5.0f, 5.0f);

    for (size_t i = 0; i < num_points; ++i) {
        float x = dist(impl_->rng);
        float y = dist(impl_->rng);
        float z = std::abs(dist(impl_->rng)) * 0.5f;
        float padding = 0.0f;

        // Add position bytes
        auto add_float = [&data](float f) {
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&f);
            for (int j = 0; j < 4; ++j) data.push_back(bytes[j]);
        };

        add_float(x);
        add_float(y);
        add_float(z);
        add_float(padding);

        // RGBA color based on height
        uint8_t r = static_cast<uint8_t>(std::clamp(z * 50.0f + 128.0f, 0.0f, 255.0f));
        uint8_t g = static_cast<uint8_t>(std::clamp(128.0f - z * 25.0f, 0.0f, 255.0f));
        uint8_t b = static_cast<uint8_t>(std::clamp(64.0f + z * 25.0f, 0.0f, 255.0f));
        uint8_t a = 255;

        data.push_back(r);
        data.push_back(g);
        data.push_back(b);
        data.push_back(a);

        // Padding to 32 bytes
        for (int j = 0; j < 12; ++j) data.push_back(0);
    }
#endif

    // Clear new data flag
    std::string topic_str(topic.data(), topic.size());
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->new_data_flags[topic_str] = false;

    return data;
}

PointCloud2Header Ros2Bridge::get_pointcloud2_header(rust::Str topic) const {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    PointCloud2Header header;
    header.timestamp_sec = static_cast<int32_t>(seconds.count());
    header.timestamp_nanosec = static_cast<uint32_t>(nanoseconds.count());
    header.frame_id = rust::String("base_link");
    header.width = 1000;
    header.height = 1;
    header.point_step = 32;
    header.row_step = 32 * 1000;
    header.is_dense = true;

    return header;
}

rust::Vec<float> Ros2Bridge::get_laserscan_ranges(rust::Str topic) const {
    rust::Vec<float> ranges;

#ifdef WIZ_ROS2_ENABLED
    // TODO: Return actual laser scan ranges
#else
    // Mock: Generate 360 range values (1 degree resolution)
    const size_t num_rays = 360;
    ranges.reserve(num_rays);

    std::uniform_real_distribution<float> noise(-0.1f, 0.1f);

    for (size_t i = 0; i < num_rays; ++i) {
        float angle = static_cast<float>(i) * M_PI / 180.0f;
        // Create some mock obstacles
        float base_range = 5.0f + 2.0f * std::sin(angle * 3.0f);
        float range = base_range + noise(impl_->rng);
        ranges.push_back(std::clamp(range, 0.1f, 30.0f));
    }
#endif

    // Clear new data flag
    std::string topic_str(topic.data(), topic.size());
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->new_data_flags[topic_str] = false;

    return ranges;
}

rust::Vec<float> Ros2Bridge::get_laserscan_intensities(rust::Str topic) const {
    rust::Vec<float> intensities;

#ifdef WIZ_ROS2_ENABLED
    // TODO: Return actual laser scan intensities
#else
    // Mock: Generate 360 intensity values
    const size_t num_rays = 360;
    intensities.reserve(num_rays);

    std::uniform_real_distribution<float> dist(0.5f, 1.0f);

    for (size_t i = 0; i < num_rays; ++i) {
        intensities.push_back(dist(impl_->rng));
    }
#endif

    return intensities;
}

LaserScanData Ros2Bridge::get_laserscan_data(rust::Str topic) const {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    LaserScanData data;
    data.timestamp_sec = static_cast<int32_t>(seconds.count());
    data.timestamp_nanosec = static_cast<uint32_t>(nanoseconds.count());
    data.frame_id = rust::String("laser_frame");
    data.angle_min = 0.0f;
    data.angle_max = static_cast<float>(2.0 * M_PI);
    data.angle_increment = static_cast<float>(M_PI / 180.0);
    data.time_increment = 0.0f;
    data.scan_time = 0.1f;
    data.range_min = 0.1f;
    data.range_max = 30.0f;

    return data;
}

bool Ros2Bridge::has_new_data(rust::Str topic) const {
    std::string topic_str(topic.data(), topic.size());
    std::lock_guard<std::mutex> lock(impl_->data_mutex);

    auto it = impl_->new_data_flags.find(topic_str);
    if (it != impl_->new_data_flags.end()) {
        return it->second;
    }
    return false;
}

TransformFfi Ros2Bridge::lookup_transform(rust::Str target_frame, rust::Str source_frame) const {
    TransformFfi transform;

#ifdef WIZ_ROS2_ENABLED
    // TODO: Use tf2_ros::Buffer to lookup transform
#else
    // Mock: Return identity or simple offset based on frame names
    std::string target(target_frame.data(), target_frame.size());
    std::string source(source_frame.data(), source_frame.size());

    transform.tx = 0.0;
    transform.ty = 0.0;
    transform.tz = 0.0;
    transform.qx = 0.0;
    transform.qy = 0.0;
    transform.qz = 0.0;
    transform.qw = 1.0; // Valid identity transform

    // Add some mock offsets
    if (source == "map" && target == "odom") {
        transform.tx = 1.0;
        transform.ty = 0.5;
    } else if (source == "odom" && target == "base_link") {
        transform.tx = 0.5;
        transform.ty = 0.2;
    } else if (source == "base_link" && target == "laser_frame") {
        transform.tx = 0.1;
        transform.tz = 0.3;
    }
#endif

    return transform;
}

rust::Vec<rust::String> Ros2Bridge::get_tf_frames() const {
    rust::Vec<rust::String> frames;

#ifdef WIZ_ROS2_ENABLED
    // TODO: Query actual TF frames
#else
    for (const auto& frame : impl_->tf_frames) {
        frames.push_back(rust::String(frame));
    }
#endif

    return frames;
}

std::unique_ptr<Ros2Bridge> create_bridge(rust::Str node_name, rust::Str namespace_) {
    return std::make_unique<Ros2Bridge>(node_name, namespace_);
}

bool transform_valid(const TransformFfi& transform) {
    // A valid quaternion has non-zero magnitude
    // For simplicity, check if qw is not zero (identity has qw=1)
    return transform.qw != 0.0 || transform.qx != 0.0 ||
           transform.qy != 0.0 || transform.qz != 0.0;
}

} // namespace wiz
