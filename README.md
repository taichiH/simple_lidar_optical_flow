# simple_lidar_optical_flow

## install
```
rosdep install -y -r --from-paths --ignore-src .
catkin build simple_lidar_optical_flow
```

## explanation

 - lidar_to_image
   - `~input_cloud` point cloud (sensor_msgs::PointCloud2)
   - `~output` image (sensor_msgs::Image)

 - flow_viewer
   - `~input_flow` flow (opencv_apps::FlowArrayStamped)
   - `~input_image` image (sensor_msgs::Image)
   - `~output` image (sensor_msgs::Image)


## launch
```
roslaunch simple_lidar_optical_flow simple_lidar_optical_flow.launch lk_flow:=false simple_flow:=true
```
