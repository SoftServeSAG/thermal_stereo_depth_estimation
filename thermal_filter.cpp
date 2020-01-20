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
#include <stdlib.h>
#include <math.h>
#include <vector>

ros::Publisher dbg_pub_true_temp;
ros::Publisher dbg_pub_true_temp_left;

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

std::vector<std::vector<int>> Logical_matrix = std::vector(32, std::vector<int>(32, 0));
std::vector<std::vector<int>> Cluster_matrix;
_Float32 Arr_2[1024];
_Float32 Temperature_matrix_2[32][32];
__int16_t Logical_matrix_2[32][32];

std::vector<std::vector<double>> centersVectorRight;
std::vector<std::vector<double>> centersVectorLeft;

bool checkCorrespondence(double centerYLeft, double centerYRight, double centerXLeft, double centerXRight, int &leftIterator, int &rightIterator)
{
  ROS_INFO("CenterXLeft: %f", centerXLeft);
  if (abs(centerYLeft - centerYRight) > 2)
    return false;
  else if (centerXLeft < 3)
  {
    leftIterator++;
    return false;
  }
  else if (centerXRight > 29)
  {
    rightIterator++;
    return false;
  }
  else
    return true;
}
std::vector<double> getDistance(std::vector<std::vector<double>> centersVectorRight, std::vector<std::vector<double>> centersVectorLeft)
{
  std::vector<double> distanceVector;
  
  

  int size = std::min(centersVectorLeft.size(), centersVectorRight.size());
  int leftIterator = 0;
  int rightIterator = 0;

  ROS_INFO("Distance function");
  

  while ((leftIterator < centersVectorLeft.size()) && (rightIterator < centersVectorRight.size()))
  { 
    ROS_INFO("Stucked");
    
    double centerYLeft = centersVectorLeft[leftIterator][0];
    double centerYRight = centersVectorRight[rightIterator][0];

    double centerXLeft = centersVectorLeft[leftIterator][1];
    double centerXRight = centersVectorRight[rightIterator][1];

    if (checkCorrespondence(centerYLeft, centerYRight, centerXLeft, centerXRight, leftIterator, rightIterator))
    {

      ROS_INFO("CenterL: %f", centerXLeft);
      ROS_INFO("CenterR: %f", centerXRight);

      if (centerXLeft == 0 || centerXRight == 0)
      {
        distanceVector.push_back(0);
      }
      else
      {
        d = abs(centerXLeft - centerXRight);
        distanceVector.push_back((F * B) / d);
      }

      

      leftIterator++;
      rightIterator++;
    }
  }

  return distanceVector;
}

std::vector<std::vector<int>> toLogicalArray(_Float32 temp_matrix[][32])
{
  std::vector<std::vector<int>> logical_vector = std::vector(32, std::vector<int>(32, 0));
  for (int i = 0; i < 32; i++)
  {
    for (int j = 0; j < 32; j++)
    {
      logical_vector[i][j] = (temp_matrix[i][j] > 0 ? -1 : 0);
    }
  }

  return logical_vector;
}

// Xmax Ymax Xmin Ymin
std::vector<std::vector<int>> toClusterArray(std::vector<std::vector<int>> logical_vector, std::vector<std::vector<int>> &coordinates_map)
{
  int clusternum = 1;
  int Xmin;
  int Xmax;
  int Ymin;
  int Ymax;

  for (int i = 0; i < 32; i++)
  {
    for (int j = 0; j < 32; j++)
    {
      int pixel = logical_vector[i][j];

      int left = 0;
      int right = 0;
      int up = 0;
      int down = 0;

      if (pixel == -1)
      {
        logical_vector[i][j] = clusternum;

        left = (i - 1) < 0 ? 0 : logical_vector[i - 1][j];
        right = (i + 1) > 31 ? 0 : logical_vector[i + 1][j];
        down = (j + 1) > 31 ? 0 : logical_vector[i][j + 1];
        up = (j - 1) < 0 ? 0 : logical_vector[i][j - 1];

        int clusterfind = std::max({left, right, down, up});

        // ROS_INFO("clusterfind %d", clusterfind);

        if (clusterfind > 0)
        {

          logical_vector[i][j] = clusterfind;
          Xmax = std::max(coordinates_map[clusterfind - 1][0], i);
          Ymax = std::max(coordinates_map[clusterfind - 1][1], j);
          Xmin = std::min(coordinates_map[clusterfind - 1][2], i);
          Ymin = std::min(coordinates_map[clusterfind - 1][3], j);

          coordinates_map[clusterfind - 1] = std::vector<int>{Xmax, Ymax, Xmin, Ymin};
        }
        else
        {
          logical_vector[i][j] = clusternum;
          coordinates_map.push_back(std::vector<int>{i, j, i, j});
          clusternum++;
        }
      }
    }
  }

  return logical_vector;
}

bool compareCoordinates(std::vector<int> v1, std::vector<int> v2)
{
  if (v1[1] < v2[1])
  {
    return true;
  }
  else if (v1[1] == v2[1])
  {
    return (v1[0] <= v2[0]);
  }
  else
    return false;
}

std::vector<std::vector<double>> getMassCenters(_Float32 temperature_matrix[32][32], std::vector<std::vector<int>> coordinates_vector)
{

  std::vector<std::vector<double>> output;

  sort(coordinates_vector.begin(), coordinates_vector.end(), compareCoordinates);

  for (int i = 0; i < coordinates_vector.size(); i++)
  {
    ROS_INFO("Xsmall: %d", coordinates_vector[i][1]);
  }

  for (int i = 0; i < coordinates_vector.size(); i++)
  {
    // get matrix center
    double cx = 0;
    double cy = 0;
    double m = 0;
    std::vector<int> current_matrix_vector = coordinates_vector[i];

    for (int x = current_matrix_vector[2]; x <= current_matrix_vector[0]; x++)
    {
      for (int y = current_matrix_vector[3]; y <= current_matrix_vector[1]; y++)
      {
        cx += temperature_matrix[x][y] * x;
        cy += temperature_matrix[x][y] * y;
        m += temperature_matrix[x][y];
      }
    }

    //those are center's the cell coordinates within the matrix
    double cmx = (cx / (m + 0.000001));
    double cmy = (cy / (m + 0.000001));

    output.push_back(std::vector<double>{cmx, cmy});
  }

  return output;
}

//Callbacks
void tempCallbackRight(const std_msgs::Float64MultiArray::ConstPtr &array)
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

  for (int i = 0; i < 32; i++)
  {
    for (int j = 0; j < 32; j++)
    {
      Temperature_matrix[i][j] = Arr[32 * i + j] < HEATMAP_RATIO * max_temp ? 0 : Arr[32 * i + j];
    }
  }

  // init new coordnates array for further interaction
  std::vector<std::vector<int>> coordinates_matrix;

  Logical_matrix = toLogicalArray(Temperature_matrix);
  Cluster_matrix = toClusterArray(Logical_matrix, coordinates_matrix);

  centersVectorRight = getMassCenters(Temperature_matrix, coordinates_matrix);

  for (int i = 0; i < centersVectorRight.size(); i++)
  {
    //whatever you'd need that value for (the position is more likely what you're after)
    double centerOfMassValue = Temperature_matrix[(int)centersVectorRight[i].at(0)][(int)centersVectorRight[i].at(1)] = 249;
  }

  cv::Mat image = cv::Mat(32, 32, CV_32FC1, Temperature_matrix);
  std_msgs::Header header_;
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_32FC1, image);

  img_bridge.toImageMsg(dbg_msg);

  // Publish to topic
  dbg_pub_true_temp.publish(dbg_msg);
  return;
}

void tempCallbackLeft(const std_msgs::Float64MultiArray::ConstPtr &array)
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

  // init new coordnates array for further interaction
  std::vector<std::vector<int>> coordinates_matrix;

  Logical_matrix = toLogicalArray(Temperature_matrix_2);
  Cluster_matrix = toClusterArray(Logical_matrix, coordinates_matrix);

  centersVectorLeft = getMassCenters(Temperature_matrix_2, coordinates_matrix);

  for (int i = 0; i < centersVectorLeft.size(); i++)
  {
    //whatever you'd need that value for (the position is more likely what you're after)
    double centerOfMassValue = Temperature_matrix_2[(int)centersVectorLeft[i].at(0)][(int)centersVectorLeft[i].at(1)] = 249;
  }

  cv::Mat image = cv::Mat(32, 32, CV_32FC1, Temperature_matrix_2);
  std_msgs::Header header_;
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_32FC1, image);

  img_bridge.toImageMsg(dbg_msg);

  // Publish to topic
  dbg_pub_true_temp_left.publish(dbg_msg);

  printf("\n");
  std::vector<double> distances = getDistance(centersVectorRight, centersVectorLeft);
  for (int i = 0; i < distances.size(); i++)
  {
    ROS_INFO("Distance %d: %f", i, distances[i]);
  }
  
  printf("\n");
  return; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  // Subscribers for temperature array

  ros::Subscriber sub_left = nh.subscribe("/uav1/thermal/middle/raw_temp_array", 1, tempCallbackRight);   // Left camera
  ros::Subscriber sub_right = nh.subscribe("/uav1/thermal/bottom/raw_temp_array", 1, tempCallbackLeft); // Right camera

  // Publishers for temperature arrays (to CV Image)
  dbg_pub_true_temp = nh.advertise<sensor_msgs::Image>("right_debug_temp", 10);
  dbg_pub_true_temp_left = nh.advertise<sensor_msgs::Image>("left_debug_temp", 10);

  ROS_INFO("START");

  ros::spin();
}