#include "custom_gridmap_costmap_generator/ObstacleFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
ObstacleFilter<T>::ObstacleFilter()
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
// Read the list of obstacles.
  param_reader.get(std::string("obstacles"), obstacles_);

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

  mapOut.add(output_layer_);
  mapOut[output_layer_] = mapOut[layer_];

  for (const auto & obstacle : obstacles_) {
    double poseX = obstacle[0];
    double poseY = obstacle[1];
    double length = obstacle[2];
    double width = obstacle[3];
    double height = obstacle[4];
    double radius = obstacle[5];
    int type = static_cast<int>(obstacle[6]);  // 0 for rectangle, 1 for circular

    if (type == 0) {
      // Rectangular obstacle
      grid_map::Polygon polygon;
      polygon.setFrameId(mapOut.getFrameId());
      polygon.addVertex(grid_map::Position(poseX - (length / 2.0), poseY + (width / 2.0)));
      polygon.addVertex(grid_map::Position(poseX + (length / 2.0), poseY + (width / 2.0)));
      polygon.addVertex(grid_map::Position(poseX + (length / 2.0), poseY - (width / 2.0)));
      polygon.addVertex(grid_map::Position(poseX - (length / 2.0), poseY - (width / 2.0)));

      // Iterate over the polygon area and apply the obstacle height
      for (grid_map::PolygonIterator iterator(mapOut, polygon);
           !iterator.isPastEnd(); ++iterator)
      {
        mapOut.at(output_layer_, *iterator) = height;
      }
    } else if (type == 1) {
      // Circular obstacle
      grid_map::Circle circle(grid_map::Position(poseX, poseY), radius);

      // Iterate over the circular area and apply the obstacle height
      for (grid_map::CircleIterator iterator(mapOut, circle);
           !iterator.isPastEnd(); ++iterator)
      {
        mapOut.at(output_layer_, *iterator) = height;
      }
    }
  }

  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::ObstacleFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)