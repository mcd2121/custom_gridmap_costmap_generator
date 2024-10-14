#include "custom_gridmap_costmap_generator/ObstacleFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
ObstacleFilter<T>::ObstacleFilter()
: length_(0.0),
  width_(1.0),
  radius_(0.5),
  poseX_(1.5),
  poseY_(1.5),
  circular_obstacle_(false)
{
    
}

template<typename T>
ObstacleFilter<T>::~ObstacleFilter()
{
}

template<typename T>
bool ObstacleFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);

  if (param_reader.get(std::string("length"), length_)) {
    circular_obstacle_ = false;
    RCLCPP_DEBUG(this->logging_interface_->get_logger(), "length_ = %f", length_);
  }
  if (param_reader.get(std::string("width"), width_)) {
    circular_obstacle_ = false;
    RCLCPP_DEBUG(this->logging_interface_->get_logger(), "width_ = %f", width_);
  }
  if (param_reader.get(std::string("radius"), radius_)) {
    circular_obstacle_ = true;
    RCLCPP_DEBUG(this->logging_interface_->get_logger(), "radius_ = %f", radius_);
  }

  if (param_reader.get(std::string("height"), height_)) {
    RCLCPP_DEBUG(this->logging_interface_->get_logger(), "height_ = %f", height_);
  }
  if (param_reader.get(std::string("poseX"), poseX_)) {
    RCLCPP_DEBUG(this->logging_interface_->get_logger(), "poseX_ = %f", poseX_);
  }
  if (param_reader.get(std::string("poseY"), poseY_)) {
    RCLCPP_DEBUG(this->logging_interface_->get_logger(), "poseY_ = %f", poseY_);
  }


  if (!param_reader.get(std::string("layer"), layer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(), "ObstacleFilter did not find parameter 'layer'.");
    return false;
  }
    
  if (!param_reader.get(std::string("output_layer"), output_layer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(), "ObstacleFilter did not find parameter 'output_layer'.");
    return false;
  }

  return true;
}

template<typename T>
bool ObstacleFilter<T>::update(const T & mapIn, T & mapOut)
{
  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(layer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(), "Check your threshold types! Type %s does not exist",
      layer_.c_str());
    return false;
  }

  // For each cell in map.  
  grid_map::Polygon polygon;
  polygon.setFrameId(mapOut.getFrameId());
  polygon.addVertex(grid_map::Position( poseX_-(length_/2.0), poseY_+(width_/2.0) ));
  polygon.addVertex(grid_map::Position( poseX_+(length_/2.0), poseY_+(width_/2.0) ));
  polygon.addVertex(grid_map::Position( poseX_+(length_/2.0), poseY_-(width_/2.0) ));
  polygon.addVertex(grid_map::Position( poseX_-(length_/2.0), poseY_-(width_/2.0) ));

  mapOut.add(output_layer_);

  mapOut[output_layer_] = mapOut[layer_];
  for (grid_map::PolygonIterator iterator(mapOut, polygon);
    !iterator.isPastEnd(); ++iterator)
  {
    mapOut.at(output_layer_, *iterator) = height_;
    
  }

  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::ObstacleFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)