#ifndef CUSTOM_GRIDMAP_COSTMAP_GENERATOR__MULTIOBSTACLEFILTER_HPP_
#define CUSTOM_GRIDMAP_COSTMAP_GENERATOR__MULTIOBSTACLEFILTER_HPP_

#include <filters/filter_base.hpp>

#include <string>
#include <vector>

namespace grid_map
{

/*!
 * Threshold filter class to set values below/above a threshold to a
 * specified value.
 */
template<typename T>
class MultiObstacleFilter : public filters::FilterBase<T>
{
public:
  /*!
   * Constructor
   */
  MultiObstacleFilter();

  /*!
   * Destructor.
   */
  virtual ~MultiObstacleFilter();

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Uses either an upper or lower threshold. If the threshold is exceeded
   * the cell value is set to the predefined value setTo_.
   * @param mapIn GridMap with the different layers to apply a threshold.
   * @param mapOut GridMap with the threshold applied to the layers.
   */
  bool update(const T & mapIn, T & mapOut) override;

private:
  //! Layer the threshold should be applied to.
  std::string layer_;
  std::string output_layer_;
  
  std::vector<std::string> obstacles_;

};

}  // namespace grid_map
#endif  // GRID_MAP_FILTERS__MultiObstacleFilter_HPP_