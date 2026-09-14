#include <rclcpp/rclcpp.hpp>

#include "rov_postpro/extract_stereo.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rov_postpro::ExtractStereo>();

  const bool live_mode = node->get_parameter("bag_path").as_string().empty();
  if (live_mode) {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
