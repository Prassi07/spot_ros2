#include <rclcpp/executors.hpp>
#include <spot_driver/local_grid/local_grid_publisher_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  spot_ros2::LocalGridPublisherNode node;

  // Spins the node with the default single-threaded executor.
  rclcpp::spin(node.get_node_base_interface());

  return 0;
}
