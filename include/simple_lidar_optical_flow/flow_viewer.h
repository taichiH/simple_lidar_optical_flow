#pragma once

#include <iostream>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv_apps/FlowArrayStamped.h>
#include <pcl_conversions/pcl_conversions.h>

namespace simple_lidar_optical_flow
{
  class FlowViewer {
  public:
    FlowViewer();
    void callback(const sensor_msgs::Image::ConstPtr& image_msg,
                  const opencv_apps::FlowArrayStamped::ConstPtr& flow_msg);

  private:
    bool check_px_range(const cv::Point& px);

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      opencv_apps::FlowArrayStamped
      > ApproximateSync;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      opencv_apps::FlowArrayStamped
      > Sync;

    boost::shared_ptr<message_filters::Synchronizer<ApproximateSync> > approximate_sync_;
    boost::shared_ptr<message_filters::Synchronizer<Sync> > sync_;

    message_filters::Subscriber<opencv_apps::FlowArrayStamped> flow_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;

    bool is_approximate_sync_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher image_pub_;

    int w_;
    int h_;
    int point_radius_;
    float z_max_;
    float z_min_;
    float vel_thresh_;

  };

} // namespace simple_lidar_optical_flow
