#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

ros::Publisher dbg_pub;
ros::Publisher dbg_pub_2;

void imageCallbackFirstCamera(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img_out;
  sensor_msgs::Image dbg_msg;

  try
  { 
    //Get pointer to thermal image
    cv_bridge::CvImageConstPtr thermal_image_pntr = cv_bridge::toCvShare(msg, "rgb8");

    // Get image Mat
    cv::Mat thermal_image = thermal_image_pntr->image;

    // Clone image before processing
    cv::Mat img_gray = thermal_image.clone();

    // Denoise image
    cv::fastNlMeansDenoising(img_gray, img_out, 10.0, 3, 6);

    // Define headers
    std_msgs::Header header_;
    header_ = msg->header;


    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::BGR8,img_out);
    img_bridge.toImageMsg(dbg_msg);

    //Publish to topic
    dbg_pub.publish(dbg_msg);
    cv::waitKey(2);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR(msg->encoding.c_str());
  }
}


void imageCallbackSecondCamera(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img_out;
  sensor_msgs::Image dbg_msg;

  try
  { 
    //Get pointer to thermal image
    cv_bridge::CvImageConstPtr thermal_image_pntr = cv_bridge::toCvShare(msg, "rgb8");

    // Get image Mat
    cv::Mat thermal_image = thermal_image_pntr->image;

    // Clone image before processing
    cv::Mat img_gray = thermal_image.clone();

    // Denoise image
    cv::fastNlMeansDenoising(img_gray, img_out, 10.0, 3, 6);

    // Define headers
    std_msgs::Header header_;
    header_ = msg->header;


    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::BGR8,img_out);
    img_bridge.toImageMsg(dbg_msg);

    //Publish to topic
    dbg_pub_2.publish(dbg_msg);
    cv::waitKey(2);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR(msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/uav1/thermal/bottom/rgb_image", 1, &imageCallbackFirstCamera);
  image_transport::Subscriber sub_2 = it.subscribe("/uav1/thermal/middle/rgb_image", 1, &imageCallbackSecondCamera);

  dbg_pub = nh.advertise<sensor_msgs::Image>("debug_image", 10);
  dbg_pub_2 = nh.advertise<sensor_msgs::Image>("debug_image_2", 10);
  ros::spin();
}