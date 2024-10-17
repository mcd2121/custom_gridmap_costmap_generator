/*
 * ImageToGridmap.cpp
 *
 *  Created on: May 4, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <string>
#include <utility>

#include "custom_gridmap_costmap_generator/ImageToGridmap.hpp"

namespace custom_gridmap_costmap_generator
{

ImageToGridmap::ImageToGridmap()
: Node("image_to_gridmap_node"),
  map_(grid_map::GridMap({"elevation"})),
  mapInitialized_(false),
  filterChain_("grid_map::GridMap")
{
  readParameters();
  map_.setBasicLayers({"elevation"});
  imageSubscriber_ =
    this->create_subscription<sensor_msgs::msg::Image>(
    imageTopic_, 1,
    std::bind(&ImageToGridmap::imageCallback, this, std::placeholders::_1));

  gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    outputTopic_, rclcpp::QoS(1).transient_local());

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

ImageToGridmap::~ImageToGridmap()
{
}

bool ImageToGridmap::readParameters()
{
  this->declare_parameter("image_topic", std::string("/image"));
  this->declare_parameter("resolution", rclcpp::ParameterValue(0.03));
  this->declare_parameter("min_height", rclcpp::ParameterValue(0.0));
  this->declare_parameter("max_height", rclcpp::ParameterValue(1.0));
  this->declare_parameter("output_topic", std::string("output"));
  this->declare_parameter("filter_chain_parameter_name", std::string("filters"));

  this->get_parameter("image_topic", imageTopic_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("min_height", minHeight_);
  this->get_parameter("max_height", maxHeight_);
  this->get_parameter("output_topic", outputTopic_);
  this->get_parameter("filter_chain_parameter_name", filterChainParametersName_);

  return true;
}

void ImageToGridmap::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!mapInitialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(*msg, resolution_, map_);
    RCLCPP_INFO(
      this->get_logger(),
      "Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
      map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    mapInitialized_ = true;
  }


  grid_map::GridMapRosConverter::addLayerFromImage(*msg, "elevation", map_, minHeight_, maxHeight_);
  grid_map::GridMapRosConverter::addColorLayerFromImage(*msg, "color", map_);

  grid_map::GridMap outputMap;
  if (!filterChain_.update(map_, outputMap)) {
    RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
    return;
  }

  // Publish as grid map.
  auto message = grid_map::GridMapRosConverter::toMessage(outputMap);
  gridMapPublisher_->publish(std::move(message));
}

}  // namespace custom_gridmap_costmap_generator
