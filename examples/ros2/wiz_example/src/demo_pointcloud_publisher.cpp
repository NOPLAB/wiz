/**
 * Demo PointCloud Publisher
 *
 * Publishes animated point cloud data for demonstrating wiz capabilities.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;

class DemoPointCloudPublisher : public rclcpp::Node
{
public:
    DemoPointCloudPublisher()
        : Node("demo_pointcloud_publisher"), tick_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/demo_pointcloud", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&DemoPointCloudPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Demo pointcloud publisher started");
    }

private:
    void timer_callback()
    {
        auto msg = sensor_msgs::msg::PointCloud2();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();

        // Number of points
        const size_t num_points = 5000;
        double time_factor = tick_ * 0.02;

        // Set up point cloud fields
        msg.height = 1;
        msg.width = num_points;
        msg.is_dense = true;
        msg.is_bigendian = false;

        // Define fields: x, y, z, rgb
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(num_points);

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

        // Generate animated spiral point cloud
        for (size_t i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / num_points;
            double angle = t * 20.0 * M_PI + time_factor;
            double radius = t * 5.0;
            double height = t * 3.0 + 0.5 * std::sin(angle * 0.5 + time_factor);

            *iter_x = static_cast<float>(radius * std::cos(angle));
            *iter_y = static_cast<float>(radius * std::sin(angle));
            *iter_z = static_cast<float>(height);

            // Color based on position (rainbow gradient)
            double hue = std::fmod(t * 360.0 + time_factor * 20.0, 360.0);
            auto [r, g, b] = hsv_to_rgb(hue, 0.9, 0.9);
            *iter_r = r;
            *iter_g = g;
            *iter_b = b;

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }

        publisher_->publish(msg);
        tick_++;
    }

    std::tuple<uint8_t, uint8_t, uint8_t> hsv_to_rgb(double h, double s, double v)
    {
        double c = v * s;
        double x = c * (1 - std::abs(std::fmod(h / 60.0, 2) - 1));
        double m = v - c;

        double r, g, b;
        if (h < 60) {
            r = c; g = x; b = 0;
        } else if (h < 120) {
            r = x; g = c; b = 0;
        } else if (h < 180) {
            r = 0; g = c; b = x;
        } else if (h < 240) {
            r = 0; g = x; b = c;
        } else if (h < 300) {
            r = x; g = 0; b = c;
        } else {
            r = c; g = 0; b = x;
        }

        return {
            static_cast<uint8_t>((r + m) * 255),
            static_cast<uint8_t>((g + m) * 255),
            static_cast<uint8_t>((b + m) * 255)
        };
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t tick_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoPointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}
