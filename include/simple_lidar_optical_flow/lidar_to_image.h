#pragma once

#include <iostream>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>

namespace simple_lidar_optical_flow
{
  class LidarToImage {
  public:
    LidarToImage();
    void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher image_pub_;
    ros::Publisher gray_image_pub_;
    ros::Subscriber cloud_sub_;

    int w_;
    int h_;
    int point_radius_;
    float z_max_;
    float z_min_;
    bool point_to_pixel
      (const pcl::PointXYZ& point, cv::Point2d& px, cv::Scalar& color);

  };

} // namespace simple_lidar_optical_flow
