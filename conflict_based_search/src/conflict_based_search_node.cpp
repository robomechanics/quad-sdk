#include <rclcpp/rclcpp.hpp>

#include "conflict_based_search/conflict_based_search.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("conflict_based_search");
  conflict_based_search::ConflictBasedSearch cbs(node);
  // run() spins inline while waiting on service futures, then returns once
  // a conflict-free solution has been published or the open list has been
  // exhausted. After it returns we keep spinning so subscribers retain
  // access to the published plans (eg. RViz subscribers, replay tools).
  cbs.run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
