#### Publish image 
ros2 run image_publisher image_publisher_node /home/hitech/ans_ws/mapping_ws/src/custom_gridmap_costmap_generator/data/willowgarage_perfect500x500.png --ros-args -p frame_id:=map
#### launch nodes 
ros2 launch custom_gridmap_costmap_generator mapping.launch.py

