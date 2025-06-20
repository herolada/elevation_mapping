#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class ElevationToOccupancyNode : public rclcpp::Node
{
public:
  ElevationToOccupancyNode()
  : Node("elevation_to_occupancy_node")
  {
    // Declare and get parameters
    min_height_ = this->declare_parameter("min_height", 0.5);
    max_height_ = this->declare_parameter("max_height", 2.0);
    occupancy_layer_ = this->declare_parameter("elevation_layer", std::string("elevation"));

    // Subscriber to elevation map
    subscription_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "elevation_map", 10,
      std::bind(&ElevationToOccupancyNode::mapCallback, this, std::placeholders::_1));

    // Publisher for occupancy grid
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "elevation_map/occupancy", rclcpp::SystemDefaultsQoS());
  }

private:
  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    // Convert ROS message to GridMap
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    if (!map.exists(occupancy_layer_)) {
      RCLCPP_WARN(this->get_logger(), "Layer '%s' does not exist in GridMap.", occupancy_layer_.c_str());
      return;
    }

    nav_msgs::msg::OccupancyGrid occupancy_grid;
    try {
      grid_map::GridMapRosConverter::toOccupancyGrid(
        map, occupancy_layer_, min_height_, max_height_, occupancy_grid);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert to OccupancyGrid: %s", e.what());
      return;
    }

    for (size_t i=0; i < occupancy_grid.data.size(); ++i) {
      if (occupancy_grid.data[i] > 0) {
        occupancy_grid.data[i] = 100;
      }
    }

    // Publish result
    publisher_->publish(occupancy_grid);
  }

  // ROS interfaces
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

  // Parameters
  double min_height_;
  double max_height_;
  std::string occupancy_layer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationToOccupancyNode>());
  rclcpp::shutdown();
  return 0;
}
