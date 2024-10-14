#ifndef CUSTOM_GRIDMAP_COSTMAP_GENERATOR__MAPPING_HPP_
#define CUSTOM_GRIDMAP_COSTMAP_GENERATOR__MAPPING_HPP_

#include <grid_map_ros/grid_map_ros.hpp>

#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace custom_gridmap_costmap_generator
{

/*!
 * Applies a chain of grid map filters to a topic and
 * republishes the resulting grid map.
 */
class Mapping : public rclcpp::Node
{
public:
  /*!
   * Constructor.
   */
  Mapping();

  /*!
   * Destructor.
   */
  virtual ~Mapping();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  /*!
   * Callback method for the incoming grid map message.
   * @param message the incoming message.
   */
  void callback(const grid_map_msgs::msg::GridMap::SharedPtr message);

private:
  //! Name of the input grid map topic.
  std::string inputTopic_;

  //! Name of the output grid map topic.
  std::string outputTopic_;

  //! Grid map subscriber
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber_;

  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name.
  std::string filterChainParametersName_;
};

}  // namespace CUSTOM_GRIDMAP_COSTMAP_GENERATOR
#endif  // CUSTOM_GRIDMAP_COSTMAP_GENERATOR__Mapping_HPP_
