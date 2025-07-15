# csf_ground_segmentation

A ROS1 (Noetic) package for ground segmentation using the Cloth Simulation Filter (CSF) method.

## Features
- Subscribes to a point cloud topic and transforms it to the `base_link` frame
- ROI filtering (z: [-2,3]m, can be adjusted)
- Uses CSF (Cloth Simulation Filter) for ground segmentation
- Publishes segmented ground and obstacle point clouds

## Topics
- **Subscribed:**
  - `/camera/depth/color/points` (sensor_msgs/PointCloud2, configurable)
- **Published:**
  - `/ground_points` (sensor_msgs/PointCloud2)
  - `/obstacle_points` (sensor_msgs/PointCloud2)

## Usage
1. Make sure you have ROS Noetic and PCL installed.
2. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
3. Run the node:
   ```bash
   rosrun csf_ground_segmentation csf_ground_segmentation_node
   ```
4. Visualize `/ground_points` and `/obstacle_points` in RViz.


## Note
- The CSF library is already embedded in this project. No additional compilation or linking is required.

## License
MIT

## Third-party Notice
This project directly includes source code from the CSF library (https://github.com/jianboqi/CSF, Apache-2.0 license). Please refer to the CSF repository for license details.