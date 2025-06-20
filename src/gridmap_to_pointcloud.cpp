#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

class GridMapToPointCloudNode : public rclcpp::Node
{
public:
  GridMapToPointCloudNode()
  : Node("gridmap_to_pointcloud_node")
  {
    // Declare and get the boolean parameter
    // this->declare_parameter<bool>("lower_bound", true);
    // lower_bound_ = this->get_parameter("lower_bound").as_bool();
    occupancy_layer_ = this->declare_parameter("elevation_layer", std::string("elevation"));

    subscription_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_map", 10,
      std::bind(&GridMapToPointCloudNode::gridMapCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/elevation_map/points", 10);
  }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);
    
    // std::string layer = lower_bound_ ? "lower_bound" : "elevation";

    sensor_msgs::msg::PointCloud2 cloud_msg;
    if (map.exists(occupancy_layer_)) {
      grid_map::GridMapRosConverter::toPointCloud(map, occupancy_layer_, cloud_msg);
      cloud_msg.header = msg->header;
      publisher_->publish(cloud_msg);
    } else {
      RCLCPP_WARN(this->get_logger(), "Layer '%s' not found in grid map!", occupancy_layer_.c_str());
    }
  }

  // bool lower_bound_;
  std::string occupancy_layer_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
