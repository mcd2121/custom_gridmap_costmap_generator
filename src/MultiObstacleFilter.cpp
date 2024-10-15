#include "custom_gridmap_costmap_generator/MultiObstacleFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
MultiObstacleFilter<T>::MultiObstacleFilter()
{
    
}

template<typename T>
MultiObstacleFilter<T>::~MultiObstacleFilter()
{
}

template<typename T>
bool MultiObstacleFilter<T>::configure()
{
  ParameterReader param_reader(this->param_prefix_, this->params_interface_);
// Read the list of obstacles.
  if (!param_reader.get(std::string("obstacles"), obstacles_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(), "MultiObstacleFilter did not find parameter 'obstacles'.");
    return false;
  }

  if (!param_reader.get(std::string("layer"), layer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(), "MultiObstacleFilter did not find parameter 'layer'.");
    return false;
  }

  if (!param_reader.get(std::string("output_layer"), output_layer_)) {
    RCLCPP_ERROR(
      this->logging_interface_->get_logger(), "MultiObstacleFilter did not find parameter 'output_layer'.");
    return false;
  }

  return true;
}

template<typename T>
bool MultiObstacleFilter<T>::update(const T & mapIn, T & mapOut)
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

  std::vector<std::string> v;
  std::string s;
  double x;
  double y;
  double l;
  double w;
  double r;
  double h;
  double slope_d;
  int fill;
  int fill_val;
  int type;
  double border;

  for (const auto & obstacle : obstacles_) {
        std::stringstream ss(obstacle);
        while (getline(ss, s, ' '))
        {
            v.push_back(s);
        }
        x = stof(v[0]);
        y = stof(v[1]);
        l = stof(v[2]);
        w = stof(v[3]);
        r = stof(v[4]);
        h = stof(v[5]);
        slope_d = stof(v[6]);
        fill = stof(v[7]);
        fill_val = stof(v[8]);
        type = stof(v[9]);
        border = stof(v[10]);
        
        std::cout<< x << " | "<<   y << " | "<<   l << " | "<<   w << " | "<<   r << " | "<<   h << " | "<<   slope_d << " | "<< fill << " | "<< fill_val << " | "<< type << " | "<< std::endl;
        
        switch (type)
        {
          case 0: {
            grid_map::Polygon polygon;
            polygon.setFrameId(mapOut.getFrameId());
            polygon.addVertex(grid_map::Position(x - (l / 2.0), y + (w / 2.0)));
            polygon.addVertex(grid_map::Position(x + (l / 2.0), y + (w / 2.0)));
            polygon.addVertex(grid_map::Position(x + (l / 2.0), y - (w / 2.0)));
            polygon.addVertex(grid_map::Position(x - (l / 2.0), y - (w / 2.0)));

            for (grid_map::PolygonIterator iterator(mapOut, polygon); !iterator.isPastEnd(); ++iterator)
            {
              mapOut.at(output_layer_, *iterator) = h;
            }

            bool success = polygon.offsetInward(border);
            if (!success) {
              RCLCPP_WARN(rclcpp::get_logger("Polygon"), "Unable to shrink the polygon by the given margin.");
              continue;
            }

            for (grid_map::PolygonIterator iterator(mapOut, polygon);
                !iterator.isPastEnd(); ++iterator)
            {
              if(fill == 0){
                mapOut.at(output_layer_, *iterator) = 0.0;
              }else if(fill == 1 ){
                mapOut.at(output_layer_, *iterator) = h;
              }else if(fill == 2 ){
                mapOut.at(output_layer_, *iterator) = NAN;
              }else{
                mapOut.at(output_layer_, *iterator) = 0.0 + ((fill_val - 0.0) / (h - 0.0)) * (0.0 - 255.0);;
              }
            }
            break;
          }

          case 1:{
            grid_map::Position center(x,y);
            for (grid_map::CircleIterator iterator(mapOut, center, r);
              !iterator.isPastEnd(); ++iterator)
            {
              mapOut.at(output_layer_, *iterator) = h;
            }
            for (grid_map::CircleIterator iterator(mapOut, center, r-border);
                !iterator.isPastEnd(); ++iterator)
            {
              if(fill == 0){
                mapOut.at(output_layer_, *iterator) = 0.0;
              }else if(fill == 1 ){
                mapOut.at(output_layer_, *iterator) = h;
              }else if(fill == 2 ){
                mapOut.at(output_layer_, *iterator) = NAN;
              }else{
                mapOut.at(output_layer_, *iterator) = 0.0 + ((fill_val - 0.0) / (h - 0.0)) * (0.0 - 255.0);;
              }
            }
            break;
          }

          default:{
            std::cout << "Type " << type << " not implimented." << std::endl; 
            break;
          }
        }
        v.clear();
  }
  return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::MultiObstacleFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)