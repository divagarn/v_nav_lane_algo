// Saves the running octomap_server's current map to a correctly-formatted
// .bt/.ot file.
//
// Exists because octomap_server's own shipped CLI tool
// (ros2 run octomap_server octomap_saver_node <path>.bt) was confirmed
// live to reject every filename tried - "Invalid file name or extension"
// regardless of a valid .bt path, whether passed as a positional argv or
// a ROS parameter, invoked via ros2 run or the installed binary directly.
// Root cause not pinned down (looks like an argv-handling bug in this
// exact ros-humble-octomap-server build).
//
// A first attempt at a Python-only replacement (writing the /octomap_binary
// service response's raw `data` field straight to a file) was also wrong:
// that field is only octomap's binary NODE data, not a complete .bt file -
// real .bt files need octomap's own text header first ("# Octomap OcTree
// binary file", id/size/res fields - confirmed by grepping the literal
// strings in liboctomap.so). Rather than hand-reconstruct that header
// format from a guess, this calls octomap's own library functions
// (octomap_msgs::binaryMsgToMap() -> AbstractOccupancyOcTree::writeBinary()),
// which are guaranteed correct since they're the same code the octree's
// native file I/O uses everywhere else.
//
// AbstractOcTree::write() (the base-class version, no "Binary" in the
// name) was tried first and is ALSO wrong for a .bt file - it always
// writes the generic "# Octomap OcTree file" header (the .ot/full-
// probability format) regardless of what extension the output filename
// happens to have, so octomap_server_static_node correctly rejected the
// result ("First line of OcTree file header does not start with
// '# Octomap OcTree binary file'") - confirmed live by round-tripping
// the saved file back through that node. writeBinary(), declared on
// AbstractOccupancyOcTree (a more specific subclass OcTree implements),
// is the one that actually writes the "# Octomap OcTree binary file"
// header .bt readers expect.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "octomap/octomap.h"
#include "octomap/AbstractOccupancyOcTree.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::fprintf(stderr, "Usage: save_octomap_node <path>.bt\n");
    return 1;
  }
  const std::string out_path = argv[1];

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("save_octomap_node");

  std::shared_ptr<octomap_msgs::msg::Octomap> received;
  auto sub = node->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap_binary", rclcpp::QoS(1).transient_local().reliable(),
    [&received](const octomap_msgs::msg::Octomap::SharedPtr msg) {
      received = msg;
    });

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (rclcpp::ok() && !received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }

  if (!received) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No message received on /octomap_binary within 10s - is octomap_server running?");
    rclcpp::shutdown();
    return 1;
  }

  octomap::AbstractOcTree * tree = octomap_msgs::binaryMsgToMap(*received);
  if (!tree) {
    RCLCPP_ERROR(node->get_logger(), "octomap_msgs::binaryMsgToMap() failed to reconstruct a tree.");
    rclcpp::shutdown();
    return 1;
  }

  auto * occupancy_tree = dynamic_cast<octomap::AbstractOccupancyOcTree *>(tree);
  if (!occupancy_tree) {
    RCLCPP_ERROR(node->get_logger(), "Reconstructed tree is not an AbstractOccupancyOcTree.");
    delete tree;
    rclcpp::shutdown();
    return 1;
  }

  bool ok = occupancy_tree->writeBinary(out_path);
  delete tree;

  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "AbstractOcTree::write(%s) failed.", out_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(), "Saved octomap (resolution=%.3fm, frame_id=%s) to %s",
    received->resolution, received->header.frame_id.c_str(), out_path.c_str());

  rclcpp::shutdown();
  return 0;
}
