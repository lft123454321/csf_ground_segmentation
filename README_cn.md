# csf_ground_segmentation

一个基于ROS1 Noetic的CSF（Cloth Simulation Filter）地面分割包。

## 功能简介
- 订阅点云话题并转换到`base_link`坐标系
- ROI区域裁剪（z: [-2,3]米，可调整）
- 使用CSF（布料模拟滤波）算法进行地面分割
- 发布分割后的地面点云和障碍物点云

## 话题说明
- **订阅：**
  - `/camera/depth/color/points`（sensor_msgs/PointCloud2，可配置）
- **发布：**
  - `/ground_points`（sensor_msgs/PointCloud2）
  - `/obstacle_points`（sensor_msgs/PointCloud2）

## 使用方法
1. 确保已安装ROS Noetic、PCL。
2. 编译包：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
3. 运行节点：
   ```bash
   rosrun csf_ground_segmentation csf_ground_segmentation_node
   ```
4. 可在RViz中可视化`/ground_points`和`/obstacle_points`。

## 说明
- 项目已经内嵌了CSF库，无需另外编译或链接。

## License
MIT

## 第三方库说明
本工程直接源码引用了CSF库（https://github.com/jianboqi/CSF，Apache-2.0 license），请参见CSF库主页获取许可协议详情。