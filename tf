Run:

rosrun tf2_ros static_transform_publisher 1 0 0 0 0 0 1 odom base_link &
rosrun tf2_ros static_transform_publisher 1 0 0 0 0 0 1 base_link laser &
rosrun tf2_ros static_transform_publisher 1 0 0 0 0 0 1 imu odom &
rosrun hokuyo_node hokuyo_node &
rosrun hector_mapping hector_mapping &
rosrun phidgets_imu phidgets_imu_node 


