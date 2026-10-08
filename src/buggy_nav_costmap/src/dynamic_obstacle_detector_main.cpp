#include <rclcpp/rclcpp.hpp>

#include "buggy_nav_costmap/dynamic_obstacle_detector.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<buggy_nav_costmap::DynamicObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
