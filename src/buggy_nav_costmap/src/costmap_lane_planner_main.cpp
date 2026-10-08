#include <rclcpp/rclcpp.hpp>

#include "buggy_nav_costmap/costmap_lane_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<buggy_nav_costmap::CostmapLanePlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
