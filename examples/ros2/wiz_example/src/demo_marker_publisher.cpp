/**
 * Demo Marker Publisher
 *
 * Publishes animated visualization markers for demonstrating wiz capabilities.
 */

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class DemoMarkerPublisher : public rclcpp::Node
{
public:
    DemoMarkerPublisher()
        : Node("demo_marker_publisher"), tick_(0)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/demo_markers", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&DemoMarkerPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Demo marker publisher started");
    }

private:
    void timer_callback()
    {
        auto msg = visualization_msgs::msg::MarkerArray();
        double time_factor = tick_ * 0.02;

        // Rotating cube
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "demo_shapes";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = 2.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.5;
            double angle = time_factor * 0.5;
            marker.pose.orientation.z = std::sin(angle / 2.0);
            marker.pose.orientation.w = std::cos(angle / 2.0);
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.r = 0.0f;
            marker.color.g = 0.8f;
            marker.color.b = 0.2f;
            marker.color.a = 0.8f;
            msg.markers.push_back(marker);
        }

        // Bouncing sphere
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "demo_shapes";
            marker.id = 1;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = -2.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 1.0 + 0.5 * std::abs(std::sin(time_factor * 2.0));
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.8;
            marker.scale.y = 0.8;
            marker.scale.z = 0.8;
            marker.color.r = 0.8f;
            marker.color.g = 0.2f;
            marker.color.b = 0.8f;
            marker.color.a = 0.9f;
            msg.markers.push_back(marker);
        }

        // Moving cylinder
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "demo_shapes";
            marker.id = 2;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = std::sin(time_factor * 0.3) * 3.0;
            marker.pose.position.y = std::cos(time_factor * 0.3) * 3.0;
            marker.pose.position.z = 0.75;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 1.5;
            marker.color.r = 0.2f;
            marker.color.g = 0.6f;
            marker.color.b = 0.9f;
            marker.color.a = 0.85f;
            msg.markers.push_back(marker);
        }

        // Arrow pointing in rotating direction
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "demo_shapes";
            marker.id = 3;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.1;
            double arrow_angle = time_factor * 0.8;
            marker.pose.orientation.z = std::sin(arrow_angle / 2.0);
            marker.pose.orientation.w = std::cos(arrow_angle / 2.0);
            marker.scale.x = 2.0;  // length
            marker.scale.y = 0.2;  // width
            marker.scale.z = 0.2;  // height
            marker.color.r = 1.0f;
            marker.color.g = 0.5f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            msg.markers.push_back(marker);
        }

        publisher_->publish(msg);
        tick_++;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t tick_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoMarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}
