#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "custom_gridmap_costmap_generator/ImageToGridmap.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_gridmap_costmap_generator::ImageToGridmap>());
  rclcpp::shutdown();
  return 0;
}
