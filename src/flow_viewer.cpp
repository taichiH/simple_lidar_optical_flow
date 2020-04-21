#include <simple_lidar_optical_flow/flow_viewer.h>

namespace simple_lidar_optical_flow
{
  FlowViewer::FlowViewer() :
    nh_(""), pnh_("~"),
    w_(500), h_(500), point_radius_(25),
    z_max_(3), z_min_(-3), vel_thresh_(0),
    is_approximate_sync_(true) {

    // flow_sub_ = pnh_.subscribe("input_flow", 1, &FlowViewer::callback, this);
    image_pub_ = pnh_.advertise<sensor_msgs::Image>("output", 1);

    pnh_.getParam("image_width", w_);
    pnh_.getParam("image_height", h_);
    pnh_.getParam("point_radius", point_radius_);
    pnh_.getParam("z_max", z_max_);
    pnh_.getParam("z_min", z_min_);
    pnh_.getParam("vel_thresh", vel_thresh_);
    pnh_.getParam("approximate_sync", is_approximate_sync_);

    flow_sub_.subscribe(pnh_, "input_flow", 1);
    image_sub_.subscribe(pnh_, "input_image", 1);

    if (is_approximate_sync_) {
      approximate_sync_ = boost::make_shared<message_filters::Synchronizer<ApproximateSync> >(1000);
      approximate_sync_->connectInput(image_sub_, flow_sub_);
      approximate_sync_->registerCallback(boost::bind(&FlowViewer::callback,this, _1, _2));
    } else {
      sync_  = boost::make_shared<message_filters::Synchronizer<Sync> >(1000);
      sync_->connectInput(image_sub_, flow_sub_);
      sync_->registerCallback(boost::bind(&FlowViewer::callback,this, _1, _2));
    }

  }

  bool FlowViewer::check_px_range(const cv::Point& px) {
    if (px.x < 0 || w_ < px.x || px.y < 0 || h_ < px.y) {
      return false;
    }
    return true;
  }

  void FlowViewer::callback(const sensor_msgs::Image::ConstPtr& image_msg,
                            const opencv_apps::FlowArrayStamped::ConstPtr& flow_msg) {
    cv::Mat image;
    try {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy
        (image_msg, "bgr8");
      image = cv_image->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Failed to convert sensor_msgs::Image to cv::Mat \n%s", e.what());
      return;
    }

    for (auto flow : flow_msg->flow) {
      cv::Point2d current_px(flow.point.x, flow.point.y);
      cv::Point2d prev_px(flow.point.x + flow.velocity.x, flow.point.y + flow.velocity.y);
      if ( !check_px_range(current_px) ) continue;
      if ( !check_px_range(prev_px) ) continue;

      if (std::fabs(flow.velocity.x) < vel_thresh_ && std::fabs(flow.velocity.y) < vel_thresh_) {
        continue;
      }

      int window_size = 3;
      bool has_valid_px = false;
      int local_radius = (window_size - 1) * 0.5;
      for (int i=prev_px.x-local_radius; i<prev_px.x-local_radius; i++) {
        for (int j=prev_px.y-local_radius; j<prev_px.y-local_radius; j++) {
          for (int c=0; c<3; c++) {
            if (image.at<cv::Vec3b>(j,i)[i] > 50) {
              has_valid_px = true;
            }
          }
        }
      }
      if ( !has_valid_px) continue;

      cv::line(image, prev_px, current_px, cv::Scalar(0,255,0), 1, 8, 0);
      cv::circle(image, current_px, 0, cv::Scalar(255,0,0), -1);
    }

    std::string txt =
      "size: " + std::to_string(w_) +
      ", point radius: " + std::to_string(point_radius_) +
      ", vel thresh: " + std::to_string(vel_thresh_);

    cv::putText(image, txt,
                cv::Point(10, 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.3, cv::Scalar(0,0,255), 1);

    sensor_msgs::ImagePtr output_image_msg = cv_bridge::CvImage
                                                  (flow_msg->header,
                                                   sensor_msgs::image_encodings::BGR8,
                                                   image).toImageMsg();
    image_pub_.publish(output_image_msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "flow_viewer");
  simple_lidar_optical_flow::FlowViewer flow_viewer;
  ros::spin();

  return 0;
}
