#include <memory>
#include <string>
#include <utility>

#include "custom_gridmap_costmap_generator/Mapping.hpp"

namespace custom_gridmap_costmap_generator
{

Mapping::Mapping()
: Node("grid_map_filters_demo"),
  filterChain_("grid_map::GridMap")
{
  if (!readParameters()) {
    return;
  }

  subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
    inputTopic_, 1,
    std::bind(&Mapping::callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    outputTopic_,
    rclcpp::QoS(1).transient_local());


  // Setup filter chain.
  if (filterChain_.configure(
      filterChainParametersName_, this->get_node_logging_interface(),
      this->get_node_parameters_interface()))
  {
    RCLCPP_INFO(this->get_logger(), "Filter chain configured.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain!");
    rclcpp::shutdown();
    return;
  }
}

Mapping::~Mapping()
{
}

bool Mapping::readParameters()
{
  this->declare_parameter<std::string>("input_topic");
  this->declare_parameter("output_topic", std::string("output"));
  this->declare_parameter("filter_chain_parameter_name", std::string("filters"));

  if (!this->get_parameter("input_topic", inputTopic_)) {
    RCLCPP_ERROR(this->get_logger(), "Could not read parameter `input_topic`.");
    return false;
  }

  this->get_parameter("output_topic", outputTopic_);
  this->get_parameter("filter_chain_parameter_name", filterChainParametersName_);
  return true;
}

void Mapping::callback(const grid_map_msgs::msg::GridMap::SharedPtr message)
{
  // Convert message to map.
  grid_map::GridMap inputMap;
  grid_map::GridMapRosConverter::fromMessage(*message, inputMap);

  // Apply filter chain.
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "PUBLISH");
  // Publish filtered output grid map.
  std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
  outputMessage = grid_map::GridMapRosConverter::toMessage(outputMap);
  publisher_->publish(std::move(outputMessage));
}

}  // namespace grid_map_demos
