#include <simple_lidar_optical_flow/lidar_to_image.h>

namespace simple_lidar_optical_flow
{
  LidarToImage::LidarToImage() :
    nh_(""), pnh_("~"), w_(500), h_(500), point_radius_(25), z_max_(3), z_min_(-3) {
    cloud_sub_ = pnh_.subscribe("input_cloud", 1, &LidarToImage::callback, this);
    image_pub_ = pnh_.advertise<sensor_msgs::Image>("output", 1);
    gray_image_pub_ = pnh_.advertise<sensor_msgs::Image>("output_gray", 1);

    pnh_.getParam("image_width", w_);
    pnh_.getParam("image_height", h_);
    pnh_.getParam("point_radius", point_radius_);
    pnh_.getParam("z_max", z_max_);
    pnh_.getParam("z_min", z_min_);
  }

  bool LidarToImage::point_to_pixel
  (const pcl::PointXYZ& point, cv::Point2d& px, cv::Scalar& color) {
    float d = 2 * point_radius_;
    px.y = std::round(-point.x * (h_ / d) + (h_ * 0.5));
    px.x = std::round(point.y * (w_ / d) + (w_ * 0.5));

    float gain = (point.z + std::abs(z_min_)) / (z_max_ - z_min_);
    int px_val_max = 255;
    int px_val_min = 0;
    int r = std::round((px_val_max - px_val_min) * gain);
    int b = std::round((px_val_max - px_val_min) * (1 - gain));
    int g = 0;
    color = cv::Scalar(b,g,r);
    return true;
  }

  void LidarToImage::callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    cv::Mat image(cv::Size(w_, h_), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat gray_image = cv::Mat::zeros(cv::Size(w_, h_), CV_8UC1);

    int cnt = 0;
    for (int i=0; i<cloud->points.size(); i++) {
      auto p = cloud->points.at(i);
      if (p.x < -point_radius_ || point_radius_ < p.x ||
          p.y < -point_radius_ || point_radius_ < p.y ||
          p.z < z_min_ || z_max_ < p.z) {
        continue;
      }
      cv::Point2d px;
      cv::Scalar color(255, 255, 255);
      point_to_pixel(p, px, color);

      cv::circle(gray_image, px, 0, color[2], -1);
      cv::circle(image, px, 0, color, -1);
    }
    sensor_msgs::ImagePtr output_image_msg = cv_bridge::CvImage
                                                  (cloud_msg->header,
                                                   sensor_msgs::image_encodings::BGR8,
                                                   image).toImageMsg();
    sensor_msgs::ImagePtr output_gray_image_msg = cv_bridge::CvImage
                                                  (cloud_msg->header,
                                                   sensor_msgs::image_encodings::MONO8,
                                                   gray_image).toImageMsg();
    image_pub_.publish(output_image_msg);
    gray_image_pub_.publish(output_gray_image_msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_lidar_optical_flow");
  simple_lidar_optical_flow::LidarToImage lidar_to_image;
  ros::spin();

  return 0;
}
