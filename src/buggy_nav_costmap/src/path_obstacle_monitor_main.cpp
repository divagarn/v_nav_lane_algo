#include <rclcpp/rclcpp.hpp>

#include "buggy_nav_costmap/path_obstacle_monitor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<buggy_nav_costmap::PathObstacleMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
