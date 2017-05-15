#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

//std::unique_ptr<image_transport::Publisher> pub;
image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("Image callback.");
  cv::Mat cv_mat;
  cv::flip(cv_bridge::toCvShare(msg, "bgr8")->image, cv_mat, -1);
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_mat).toImageMsg();
  pub.publish(out_msg);
  // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  pub = it.advertise("/uvc_camera/cam_1_rot/image_raw", 100);
  image_transport::Subscriber sub =
      it.subscribe("/uvc_camera/cam_1/image_raw", 100, imageCallback);
  ros::spin();
}
