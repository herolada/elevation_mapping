#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class PosePublisherNode : public rclcpp::Node
{
public:
    PosePublisherNode()
        : Node("pose_publisher_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        this->declare_parameter("map_frame_id", std::string("gps_odom"));
        this->get_parameter("map_frame_id", mapFrameId_);
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("base_link_pose", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PosePublisherNode::publish_pose, this));
    }

private:
    void publish_pose()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            // Lookup transform from "odom" to "base_link"
            transform_stamped = tf_buffer_.lookupTransform(
                mapFrameId_, "base_link", tf2::TimePointZero);

            nav_msgs::msg::Odometry pose_msg;
            pose_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
            pose_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
            pose_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
            pose_msg.pose.pose.orientation = transform_stamped.transform.rotation;
            pose_msg.pose.covariance[0] = 0.1; 
            pose_msg.pose.covariance[7] = 0.1; 
            pose_msg.pose.covariance[14] = 0.1; 
            pose_msg.pose.covariance[21] = 0.1; 
            pose_msg.pose.covariance[28] = 0.1; 
            pose_msg.pose.covariance[35] = 0.1; 

            publisher_->publish(pose_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
        }
    }

    std::string mapFrameId_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
