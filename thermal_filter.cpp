#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <math.h>

ros::Publisher dbg_pub;
ros::Publisher dbg_pub_2;
ros::Publisher dbg_pub_true_temp;
ros::Publisher dbg_pub_true_temp_2;

// --------- Parameters ------------

const double HEATMAP_RATIO = 0.80;
const double START_TEMPERATURE = 50;

// Depth estimation parameters
// Distance between cameras
const double B = 0.20;
// Focal parameter
const double F = 55;
// Init distance
double d = 0;

// ---------------------------------

_Float32 Arr[1024];
_Float32 Temperature_matrix[32][32];

_Float32 Arr_2[1024];
_Float32 Temperature_matrix_2[32][32];

void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr &array);
void arrayCallback_2(const std_msgs::Float64MultiArray::ConstPtr &array);
double center_1;
double center_2;

// double printCalibrationParameter() {
//   double d = 0;

//   if(center_1==0 || center_2 ==0) {
//     return 0;
//   } else {
//     d=abs(center_1-center_2);
//   }

//   return (2 * d)/B;
// }

double getDistance() {

  if(center_1==0 || center_2 ==0) {
    return 0;
  } else {
    d=abs(center_1-center_2);
  }

  return (F*B)/d;
}

std::vector<double> getMassCenters(temperature_matrix){
  // get matrix center
  double cx = 0;
  double cy = 0;
  double m = 0;

  for (int x = 0; x < 32; x++)
  {
    for (int y = 0; y < 32; y++)
    {
      cx += Temperature_matrix[x][y] * x;
      cy += Temperature_matrix[x][y] * y;
      m += Temperature_matrix[x][y];
    }
  }

  //those are center's the cell coordinates within the matrix
  int cmx = (cx / (m + 0.00001));
  int cmy = (cy / (m + 0.00001));

  return std::vector<double> ({cmx,cmy});
}


void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr &array)
{

  int i = 0;
  int j = 0;
  sensor_msgs::Image dbg_msg;
  float max_temp = 0;

  // set all the remaining numbers
  for (std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
    Arr[i] = *it > START_TEMPERATURE ? *it : 0;
    if (max_temp < *it)
    {
      max_temp = *it;
    }
    i++;
  }

  // Print it
  // printf("\n");
  // printf("%f", max_temp);
  // printf("-----------------------------------------------------------------");

  for (int i = 0; i < 32; i++)
  {
    for (int j = 0; j < 32; j++)
    {
      //printf("%d", Arr[32*i + j]);
      Temperature_matrix[i][j] = Arr[32 * i + j] < HEATMAP_RATIO * max_temp ? 0 : Arr[32 * i + j];
      // printf("%f", Temperature_matrix[i][j]);
      // printf(" ");
    }
    // printf("\n");
  }

  // printf("\n");
  // printf("-----------------------------------------------------------------");
  // printf("\n");

  // get matrix center
  double cx = 0;
  double cy = 0;
  double m = 0;

  for (int x = 0; x < 32; x++)
  {
    for (int y = 0; y < 32; y++)
    {
      cx += Temperature_matrix[x][y] * x;
      cy += Temperature_matrix[x][y] * y;
      m += Temperature_matrix[x][y];
    }
  }

  //those are center's the cell coordinates within the matrix
  int cmx = (int)(cx / (m + 0.00001));
  int cmy = (int)(cy / (m + 0.00001));

  printf("\n");
  printf("CMY1: ");
  printf("%d",cmy);
  printf("\n");
  center_1 = (cy / (m + 0.00001));

  //whatever you'd need that value for (the position is more likely what you're after)
  double centerOfMassValue = Temperature_matrix[cmx][cmy] = 249;

  cv::Mat image = cv::Mat(32, 32, CV_32FC1, Temperature_matrix);
  std_msgs::Header header_;
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_32FC1, image);

  img_bridge.toImageMsg(dbg_msg);

  // Publish to topic
  dbg_pub_true_temp.publish(dbg_msg);
  return;
}


void arrayCallback_2(const std_msgs::Float64MultiArray::ConstPtr &array)
{

  int i = 0;
  int j = 0;
  sensor_msgs::Image dbg_msg;
  float max_temp = 0;

  // set all the remaining numbers
  for (std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
    Arr_2[i] = *it > START_TEMPERATURE ? *it : 0;
    if (max_temp < *it)
    {
      max_temp = *it;
    }
    i++;
  }

  for (int i = 0; i < 32; i++)
  {
    for (int j = 0; j < 32; j++)
    {
      Temperature_matrix_2[i][j] = Arr_2[32 * i + j] < HEATMAP_RATIO * max_temp ? 0 : Arr_2[32 * i + j];
    }
  }

  // get matrix center
  double cx = 0;
  double cy = 0;
  double m = 0;

  for (int x = 0; x < 32; x++)
  {
    for (int y = 0; y < 32; y++)
    {
      cx += Temperature_matrix_2[x][y] * x;
      cy += Temperature_matrix_2[x][y] * y;
      m += Temperature_matrix_2[x][y];
    }
  }

  //those are center's the cell coordinates within the matrix
  int cmx = (int)(cx / (m + 0.00001));
  int cmy = (int)(cy / (m + 0.00001));

  printf("\n");
  printf("CMY2: ");
  printf("%d",cmy);
  printf("\n");
  center_2 = (cy / (m + 0.00001));

  //whatever you'd need that value for (the position is more likely what you're after)
  double centerOfMassValue = Temperature_matrix_2[cmx][cmy] = 249;

  cv::Mat image = cv::Mat(32, 32, CV_32FC1, Temperature_matrix_2);
  std_msgs::Header header_;
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_32FC1, image);

  img_bridge.toImageMsg(dbg_msg);

  // Publish to topic
  dbg_pub_true_temp_2.publish(dbg_msg);

  printf("\n");
  printf("%f",getDistance());
  printf("\n");
  return;
}


void imageCallbackFirstCamera(const sensor_msgs::ImageConstPtr &msg)
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

    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::BGR8, img_out);
    img_bridge.toImageMsg(dbg_msg);

    //Publish to topic
    dbg_pub.publish(dbg_msg);
    cv::waitKey(2);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR(msg->encoding.c_str());
  }
}

void imageCallbackSecondCamera(const sensor_msgs::ImageConstPtr &msg)
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

    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::BGR8, img_out);
    img_bridge.toImageMsg(dbg_msg);

    //Publish to topic
    dbg_pub_2.publish(dbg_msg);
    cv::waitKey(2);
  }
  catch (cv_bridge::Exception &e)
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

  // Subscribers for temperature array
  ros::Subscriber sub_3 = nh.subscribe("/uav1/thermal/bottom/raw_temp_array", 10, arrayCallback);
  ros::Subscriber sub_4 = nh.subscribe("/uav1/thermal/middle/raw_temp_array", 10, arrayCallback_2);

  // Publishers for noise-reduces images
  dbg_pub = nh.advertise<sensor_msgs::Image>("debug_image", 10);
  dbg_pub_2 = nh.advertise<sensor_msgs::Image>("debug_image_2", 10);

  // Publishers for temperature arrays (to CV Image)
  dbg_pub_true_temp = nh.advertise<sensor_msgs::Image>("debug_temp", 10);
  dbg_pub_true_temp_2 = nh.advertise<sensor_msgs::Image>("debug_temp_2", 10);

  ros::spin();
}