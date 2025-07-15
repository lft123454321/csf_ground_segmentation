#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
// 集成CSF库头文件
#include "csf/CSF.h"
#include "csf/point_cloud.h"

class CSFGroundSegmentation {
public:
    CSFGroundSegmentation(ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
        sub_ = nh.subscribe("/camera/depth/color/points", 1, &CSFGroundSegmentation::cloudCallback, this);
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 1);
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1);
        // 读取私有参数
        pnh_.param("leaf_size", leaf_size_, 0.01f);
        pnh_.param("bSloopSmooth", bSloopSmooth_, true);
        pnh_.param("class_threshold", class_threshold_, 0.2);
        pnh_.param("cloth_resolution", cloth_resolution_, 0.1);
        pnh_.param("interations", interations_, 500);
        pnh_.param("rigidness", rigidness_, 3);
        pnh_.param("time_step", time_step_, 0.65);
        pnh_.param("use_voxel_grid", use_voxel_grid_, true);
        pnh_.param("use_outlier_removal", use_outlier_removal_, false);
        pnh_.param("grid_size", grid_size_, 0.5f);
        pnh_.param("min_x", min_x_, 0.0f);
        pnh_.param("max_x", max_x_, 6.0f);
        pnh_.param("min_y", min_y_, -5.0f);
        pnh_.param("max_y", max_y_, 5.0f);
        pnh_.param("min_z", min_z_, -1.0f);
        pnh_.param("max_z", max_z_, 2.0f);
        pnh_.param("min_points_per_grid", min_points_per_grid_, 5);
        pnh_.param("stddev_thresh", stddev_thresh_, 2.0f);
        pnh_.param("hist_bin_size", hist_bin_size_, 0.1f);
        pnh_.param("hist_ratio_thresh", hist_ratio_thresh_, 0.2f);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        auto t1 = std::chrono::high_resolution_clock::now();
        // Transform to base_link
        sensor_msgs::PointCloud2 cloud_base;
        try {
            listener_.waitForTransform("base_link", msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
            pcl_ros::transformPointCloud("base_link", *msg, cloud_base, listener_);
        } catch (tf::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_base, *cloud);

        // ROI裁剪
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x"); pass.setFilterLimits(min_x_, max_x_); pass.filter(*cloud_roi);
        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("y"); pass.setFilterLimits(min_y_, max_y_); pass.filter(*cloud_roi);
        pass.setInputCloud(cloud_roi);
        pass.setFilterFieldName("z"); pass.setFilterLimits(min_z_, max_z_); pass.filter(*cloud_roi);
        // VoxelGrid降采样（可选）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
        if (use_voxel_grid_) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(cloud_roi);
            voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            voxel.filter(*cloud_ds);
        } else {
            *cloud_ds = *cloud_roi;
        }

        // 离群点剔除
        if (use_outlier_removal_) {
            cloud_ds = gridOutlierRemoval(cloud_ds);
        }

        // --- CSF 地面分割 ---
        // 1. 转换为CSF库的点云格式
        csf::PointCloud csf_cloud;
        csf_cloud.reserve(cloud_ds->size());
        for (const auto& pt : cloud_ds->points) {
            csf_cloud.emplace_back(pt.x, pt.y, pt.z);
        }

        // 2. 创建CSF对象并设置参数（由rosparam控制）
        CSF csf;
        csf.setPointCloud(csf_cloud);
        csf.params.bSloopSmooth = bSloopSmooth_;
        csf.params.class_threshold = class_threshold_;
        csf.params.cloth_resolution = cloth_resolution_;
        csf.params.interations = interations_;
        csf.params.rigidness = rigidness_;
        csf.params.time_step = time_step_;

        // 3. 调用CSF分割
        std::vector<int> ground_indices, non_ground_indices;
        csf.do_filtering(ground_indices, non_ground_indices, false);

        // 4. 提取地面/非地面点云
        pcl::PointIndices::Ptr ground_idx(new pcl::PointIndices);
        pcl::PointIndices::Ptr obstacle_idx(new pcl::PointIndices);
        ground_idx->indices = ground_indices;
        obstacle_idx->indices = non_ground_indices;

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_ds);
        extract.setIndices(ground_idx);
        extract.setNegative(false);
        extract.filter(*ground);
        extract.setIndices(obstacle_idx);
        extract.setNegative(false);
        extract.filter(*obstacle);

        // 5. 发布ROS消息
        sensor_msgs::PointCloud2 ground_msg, obstacle_msg;
        pcl::toROSMsg(*ground, ground_msg);
        pcl::toROSMsg(*obstacle, obstacle_msg);
        ground_msg.header = cloud_base.header;
        obstacle_msg.header = cloud_base.header;
        ground_pub_.publish(ground_msg);
        obstacle_pub_.publish(obstacle_msg);

        auto t2 = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        ROS_INFO("Frame processed in %.2f ms", elapsed_ms);

    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_;
    ros::Publisher ground_pub_;
    ros::Publisher obstacle_pub_;
    tf::TransformListener listener_;
    float leaf_size_;
    bool use_voxel_grid_;
    bool use_outlier_removal_;
    // CSF参数
    bool bSloopSmooth_;
    double class_threshold_;
    double cloth_resolution_;
    int interations_;
    int rigidness_;
    double time_step_;
    // 新增成员变量
    float grid_size_;
    float min_x_;
    float max_x_;
    float min_y_;
    float max_y_;
    float min_z_;
    float max_z_;
    int min_points_per_grid_;
    float stddev_thresh_;
    float hist_bin_size_ = 0.1f;
    float hist_ratio_thresh_ = 0.2f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr gridOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
        const int grid_x_num = static_cast<int>((max_x_ - min_x_) / grid_size_);
        const int grid_y_num = static_cast<int>((max_y_ - min_y_) / grid_size_);
        std::vector<std::vector<std::vector<float>>> grid_z(grid_x_num, std::vector<std::vector<float>>(grid_y_num));
        for (const auto& pt : input->points) {
            int ix = static_cast<int>((pt.x - min_x_) / grid_size_);
            int iy = static_cast<int>((pt.y - min_y_) / grid_size_);
            if (ix >= 0 && ix < grid_x_num && iy >= 0 && iy < grid_y_num) {
                grid_z[ix][iy].push_back(pt.z);
            }
        }
        std::vector<bool> keep(input->size(), true);
        for (int ix = 0; ix < grid_x_num; ++ix) {
            for (int iy = 0; iy < grid_y_num; ++iy) {
                auto& zlist = grid_z[ix][iy];
                if (zlist.empty()) continue;
                // 统计z直方图
                float min_z = *std::min_element(zlist.begin(), zlist.end());
                float max_z = *std::max_element(zlist.begin(), zlist.end());
                int bin_num = std::max(1, static_cast<int>((max_z - min_z) / hist_bin_size_) + 1);
                std::vector<int> hist(bin_num, 0);
                for (float z : zlist) {
                    int bin = std::min(bin_num - 1, static_cast<int>((z - min_z) / hist_bin_size_));
                    hist[bin]++;
                }
                // 找到点数最多的bin
                int max_bin = std::distance(hist.begin(), std::max_element(hist.begin(), hist.end()));
                int max_bin_count = hist[max_bin];
                // 标记需要移除的bin
                std::vector<bool> bin_remove(bin_num, false);
                for (int b = 0; b < max_bin; ++b) {
                    if (hist[b] < hist_ratio_thresh_ * max_bin_count) {
                        bin_remove[b] = true;
                    }
                }
                // 标记需要移除的点
                for (size_t i = 0; i < input->size(); ++i) {
                    const auto& pt = input->points[i];
                    int cx = static_cast<int>((pt.x - min_x_) / grid_size_);
                    int cy = static_cast<int>((pt.y - min_y_) / grid_size_);
                    if (cx == ix && cy == iy) {
                        int bin = std::min(bin_num - 1, static_cast<int>((pt.z - min_z) / hist_bin_size_));
                        if (bin_remove[bin]) keep[i] = false;
                    }
                }
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        filtered->reserve(input->size());
        for (size_t i = 0; i < input->size(); ++i) {
            if (keep[i]) filtered->push_back(input->points[i]);
        }
        return filtered;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "csf_ground_segmentation_node");
    ros::NodeHandle nh;
    CSFGroundSegmentation node(nh);
    ros::spin();
    return 0;
}
