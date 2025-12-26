/**
 * Demo TF Broadcaster
 *
 * Broadcasts animated TF frames for demonstrating wiz TF visualization.
 */

#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class DemoTFBroadcaster : public rclcpp::Node
{
public:
    DemoTFBroadcaster()
        : Node("demo_tf_broadcaster"), tick_(0)
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&DemoTFBroadcaster::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Demo TF broadcaster started");
    }

private:
    void timer_callback()
    {
        double time_factor = tick_ * 0.02;
        auto now = this->now();

        // map -> odom (static for this demo)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "map";
            t.child_frame_id = "odom";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t);
        }

        // odom -> base_link (moving robot)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";

            // Figure-8 path
            double path_t = time_factor * 0.2;
            t.transform.translation.x = 2.0 * std::sin(path_t);
            t.transform.translation.y = std::sin(path_t * 2.0);
            t.transform.translation.z = 0.0;

            // Orientation along path
            double heading = std::atan2(
                2.0 * std::cos(path_t * 2.0),
                2.0 * std::cos(path_t)
            );
            t.transform.rotation.z = std::sin(heading / 2.0);
            t.transform.rotation.w = std::cos(heading / 2.0);

            tf_broadcaster_->sendTransform(t);
        }

        // base_link -> laser_frame (sensor mount)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "base_link";
            t.child_frame_id = "laser_frame";
            t.transform.translation.x = 0.1;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.2;
            t.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t);
        }

        // base_link -> camera_frame (sensor mount with slight rotation)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "base_link";
            t.child_frame_id = "camera_frame";
            t.transform.translation.x = 0.15;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.3;
            // Tilt down slightly
            double tilt = -0.1;
            t.transform.rotation.y = std::sin(tilt / 2.0);
            t.transform.rotation.w = std::cos(tilt / 2.0);
            tf_broadcaster_->sendTransform(t);
        }

        // base_link -> arm_base (robot arm base)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "base_link";
            t.child_frame_id = "arm_base";
            t.transform.translation.x = -0.1;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.5;
            t.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t);
        }

        // arm_base -> arm_link1 (rotating arm segment)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "arm_base";
            t.child_frame_id = "arm_link1";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.1;
            // Rotating joint
            double joint_angle = std::sin(time_factor * 0.5) * 0.8;
            t.transform.rotation.z = std::sin(joint_angle / 2.0);
            t.transform.rotation.w = std::cos(joint_angle / 2.0);
            tf_broadcaster_->sendTransform(t);
        }

        // arm_link1 -> arm_link2 (second arm segment)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "arm_link1";
            t.child_frame_id = "arm_link2";
            t.transform.translation.x = 0.3;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            // Rotating joint
            double joint_angle = std::sin(time_factor * 0.7 + 1.0) * 0.6;
            t.transform.rotation.y = std::sin(joint_angle / 2.0);
            t.transform.rotation.w = std::cos(joint_angle / 2.0);
            tf_broadcaster_->sendTransform(t);
        }

        // arm_link2 -> end_effector (gripper)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "arm_link2";
            t.child_frame_id = "end_effector";
            t.transform.translation.x = 0.25;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t);
        }

        tick_++;
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t tick_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
