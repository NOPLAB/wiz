// Include cxx-generated header first (contains struct definitions)
#include "wiz-server/src/ffi.rs.h"
#include "wiz-server/cxx/bridge.hpp"

#include <cmath>
#include <map>
#include <mutex>
#include <random>
#include <chrono>

#ifdef WIZ_ROS2_ENABLED
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#endif

namespace wiz {

// Implementation details
struct Ros2Bridge::Impl {
    std::string node_name;
    std::string namespace_;
    bool initialized = false;

#ifdef WIZ_ROS2_ENABLED
    std::shared_ptr<rclcpp::Node> node;
    static bool rclcpp_initialized;

    // ROS2 subscriptions
    mutable std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> ros2_subscriptions;

    // Stored messages from ROS2 callbacks
    mutable std::map<std::string, sensor_msgs::msg::LaserScan::ConstSharedPtr> laserscan_msgs;
    mutable std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointcloud2_msgs;

    // TF2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
#endif

    // Mock subscriptions (used when ROS2 is disabled)
    mutable std::map<std::string, std::string> subscriptions; // topic -> msg_type

    // Data storage
    mutable std::mutex data_mutex;
    mutable std::map<std::string, bool> new_data_flags;

    // Mock TF frames
    std::vector<std::string> tf_frames = {"map", "odom", "base_link", "laser_frame"};

    // Random generator for mock data
    mutable std::mt19937 rng{std::random_device{}()};

    Impl(const std::string& name, const std::string& ns)
        : node_name(name), namespace_(ns) {
#ifdef WIZ_ROS2_ENABLED
        // Initialize rclcpp if not already done
        if (!rclcpp_initialized) {
            rclcpp::init(0, nullptr);
            rclcpp_initialized = true;
        }
        // Create the node
        node = std::make_shared<rclcpp::Node>(name, ns);

        // Initialize TF2 buffer and listener
        tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
#endif
        initialized = true;
    }

    ~Impl() {
#ifdef WIZ_ROS2_ENABLED
        // Clear subscriptions first
        ros2_subscriptions.clear();
        tf_listener.reset();
        tf_buffer.reset();
        node.reset();
        // Note: We don't call rclcpp::shutdown() here as other bridges might exist
#endif
    }
};

#ifdef WIZ_ROS2_ENABLED
bool Ros2Bridge::Impl::rclcpp_initialized = false;
#endif

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
    if (impl_->node) {
        rclcpp::spin_some(impl_->node);
    }
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
    if (impl_->node) {
        // Query actual topics from ROS2
        auto topic_names_and_types = impl_->node->get_topic_names_and_types();
        for (const auto& [name, types] : topic_names_and_types) {
            // Use the first type if multiple types are available
            std::string msg_type = types.empty() ? "unknown" : types[0];
            topics.push_back(TopicInfoFfi{
                rust::String(name),
                rust::String(msg_type)
            });
        }
    }
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
    // Check if already subscribed
    {
        std::lock_guard<std::mutex> lock(impl_->data_mutex);
        if (impl_->ros2_subscriptions.count(topic_str) > 0) {
            return true; // Already subscribed
        }
    }

    // Create ROS2 subscription with callback
    auto callback = [this, topic_str](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(impl_->data_mutex);
        impl_->pointcloud2_msgs[topic_str] = msg;
        impl_->new_data_flags[topic_str] = true;
    };

    auto sub = impl_->node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_str, rclcpp::SensorDataQoS(), callback);

    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->ros2_subscriptions[topic_str] = sub;
    impl_->subscriptions[topic_str] = "sensor_msgs/msg/PointCloud2";
    impl_->new_data_flags[topic_str] = false;
    return true;
#else
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->subscriptions[topic_str] = "sensor_msgs/msg/PointCloud2";
    impl_->new_data_flags[topic_str] = false;
    return true;
#endif
}

bool Ros2Bridge::subscribe_laserscan(rust::Str topic) const {
    std::string topic_str(topic.data(), topic.size());

#ifdef WIZ_ROS2_ENABLED
    // Check if already subscribed
    {
        std::lock_guard<std::mutex> lock(impl_->data_mutex);
        if (impl_->ros2_subscriptions.count(topic_str) > 0) {
            return true; // Already subscribed
        }
    }

    // Create ROS2 subscription with callback
    auto callback = [this, topic_str](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(impl_->data_mutex);
        impl_->laserscan_msgs[topic_str] = msg;
        impl_->new_data_flags[topic_str] = true;
    };

    auto sub = impl_->node->create_subscription<sensor_msgs::msg::LaserScan>(
        topic_str, rclcpp::SensorDataQoS(), callback);

    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->ros2_subscriptions[topic_str] = sub;
    impl_->subscriptions[topic_str] = "sensor_msgs/msg/LaserScan";
    impl_->new_data_flags[topic_str] = false;
    return true;
#else
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->subscriptions[topic_str] = "sensor_msgs/msg/LaserScan";
    impl_->new_data_flags[topic_str] = false;
    return true;
#endif
}

bool Ros2Bridge::unsubscribe(rust::Str topic) const {
    std::string topic_str(topic.data(), topic.size());

    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    auto it = impl_->subscriptions.find(topic_str);
    if (it != impl_->subscriptions.end()) {
        impl_->subscriptions.erase(it);
        impl_->new_data_flags.erase(topic_str);
#ifdef WIZ_ROS2_ENABLED
        impl_->ros2_subscriptions.erase(topic_str);
        impl_->laserscan_msgs.erase(topic_str);
        impl_->pointcloud2_msgs.erase(topic_str);
#endif
        return true;
    }
    return false;
}

rust::Vec<uint8_t> Ros2Bridge::get_pointcloud2_data(rust::Str topic) const {
    rust::Vec<uint8_t> data;
    std::string topic_str(topic.data(), topic.size());

#ifdef WIZ_ROS2_ENABLED
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    auto it = impl_->pointcloud2_msgs.find(topic_str);
    if (it != impl_->pointcloud2_msgs.end() && it->second) {
        const auto& msg = it->second;
        // Copy the raw point cloud data
        data.reserve(msg->data.size());
        for (const auto& byte : msg->data) {
            data.push_back(byte);
        }
    }
    impl_->new_data_flags[topic_str] = false;
    return data;
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

    // Clear new data flag
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->new_data_flags[topic_str] = false;

    return data;
#endif
}

PointCloud2Header Ros2Bridge::get_pointcloud2_header(rust::Str topic) const {
    std::string topic_str(topic.data(), topic.size());
    PointCloud2Header header;

#ifdef WIZ_ROS2_ENABLED
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    auto it = impl_->pointcloud2_msgs.find(topic_str);
    if (it != impl_->pointcloud2_msgs.end() && it->second) {
        const auto& msg = it->second;
        header.timestamp_sec = static_cast<int32_t>(msg->header.stamp.sec);
        header.timestamp_nanosec = msg->header.stamp.nanosec;
        header.frame_id = rust::String(msg->header.frame_id);
        header.width = msg->width;
        header.height = msg->height;
        header.point_step = msg->point_step;
        header.row_step = msg->row_step;
        header.is_dense = msg->is_dense;
    } else {
        // No data yet, return empty header with current timestamp
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
        header.timestamp_sec = static_cast<int32_t>(seconds.count());
        header.timestamp_nanosec = static_cast<uint32_t>(nanoseconds.count());
        header.frame_id = rust::String("base_link");
        header.width = 0;
        header.height = 0;
        header.point_step = 0;
        header.row_step = 0;
        header.is_dense = true;
    }
    return header;
#else
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    header.timestamp_sec = static_cast<int32_t>(seconds.count());
    header.timestamp_nanosec = static_cast<uint32_t>(nanoseconds.count());
    header.frame_id = rust::String("base_link");
    header.width = 1000;
    header.height = 1;
    header.point_step = 32;
    header.row_step = 32 * 1000;
    header.is_dense = true;

    return header;
#endif
}

rust::Vec<float> Ros2Bridge::get_laserscan_ranges(rust::Str topic) const {
    rust::Vec<float> ranges;
    std::string topic_str(topic.data(), topic.size());

#ifdef WIZ_ROS2_ENABLED
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    auto it = impl_->laserscan_msgs.find(topic_str);
    if (it != impl_->laserscan_msgs.end() && it->second) {
        const auto& msg = it->second;
        ranges.reserve(msg->ranges.size());
        for (const auto& r : msg->ranges) {
            ranges.push_back(r);
        }
    }
    impl_->new_data_flags[topic_str] = false;
    return ranges;
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

    // Clear new data flag
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    impl_->new_data_flags[topic_str] = false;

    return ranges;
#endif
}

rust::Vec<float> Ros2Bridge::get_laserscan_intensities(rust::Str topic) const {
    rust::Vec<float> intensities;
    std::string topic_str(topic.data(), topic.size());

#ifdef WIZ_ROS2_ENABLED
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    auto it = impl_->laserscan_msgs.find(topic_str);
    if (it != impl_->laserscan_msgs.end() && it->second) {
        const auto& msg = it->second;
        intensities.reserve(msg->intensities.size());
        for (const auto& i : msg->intensities) {
            intensities.push_back(i);
        }
    }
    return intensities;
#else
    // Mock: Generate 360 intensity values
    const size_t num_rays = 360;
    intensities.reserve(num_rays);

    std::uniform_real_distribution<float> dist(0.5f, 1.0f);

    for (size_t i = 0; i < num_rays; ++i) {
        intensities.push_back(dist(impl_->rng));
    }

    return intensities;
#endif
}

LaserScanData Ros2Bridge::get_laserscan_data(rust::Str topic) const {
    std::string topic_str(topic.data(), topic.size());
    LaserScanData data;

#ifdef WIZ_ROS2_ENABLED
    std::lock_guard<std::mutex> lock(impl_->data_mutex);
    auto it = impl_->laserscan_msgs.find(topic_str);
    if (it != impl_->laserscan_msgs.end() && it->second) {
        const auto& msg = it->second;
        data.timestamp_sec = static_cast<int32_t>(msg->header.stamp.sec);
        data.timestamp_nanosec = msg->header.stamp.nanosec;
        data.frame_id = rust::String(msg->header.frame_id);
        data.angle_min = msg->angle_min;
        data.angle_max = msg->angle_max;
        data.angle_increment = msg->angle_increment;
        data.time_increment = msg->time_increment;
        data.scan_time = msg->scan_time;
        data.range_min = msg->range_min;
        data.range_max = msg->range_max;
    } else {
        // No data yet, return default values
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
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
    }
    return data;
#else
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

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
#endif
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
    std::string target(target_frame.data(), target_frame.size());
    std::string source(source_frame.data(), source_frame.size());

#ifdef WIZ_ROS2_ENABLED
    try {
        auto tf = impl_->tf_buffer->lookupTransform(target, source, tf2::TimePointZero);
        transform.tx = tf.transform.translation.x;
        transform.ty = tf.transform.translation.y;
        transform.tz = tf.transform.translation.z;
        transform.qx = tf.transform.rotation.x;
        transform.qy = tf.transform.rotation.y;
        transform.qz = tf.transform.rotation.z;
        transform.qw = tf.transform.rotation.w;
    } catch (const tf2::TransformException& ex) {
        // Transform not available, return identity
        transform.tx = 0.0;
        transform.ty = 0.0;
        transform.tz = 0.0;
        transform.qx = 0.0;
        transform.qy = 0.0;
        transform.qz = 0.0;
        transform.qw = 1.0;
    }
#else
    // Mock: Return identity or simple offset based on frame names
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
    // Get all known frames from TF2 buffer
    std::vector<std::string> frame_list;
    impl_->tf_buffer->_getFrameStrings(frame_list);
    for (const auto& frame : frame_list) {
        frames.push_back(rust::String(frame));
    }
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
