#include "thermal_stereo.hpp"

namespace thermal_stereo
{

void ThermalStereo::tempCallbackLeft(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if (!is_initialized_)
    return;

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_left_thermal_counters_);
    got_left_thermal_data_ = true;
    time_last_left_thermal_data_ = ros::Time::now();
    got_new_left_thermal_data_ = true;
  }

  {
    std::scoped_lock lock(mutex_left_thermal_data_);
    left_thermal_layout_ = msg->layout;
    //left_thermal_data_ = std::make_unique<std_msgs::Float64 []>(msg->data);
    left_thermal_data_ = msg->data;
  }
  ROS_INFO("Left Message");
}

void ThermalStereo::tempCallbackRight(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if (!is_initialized_)
    return;

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_right_thermal_counters_);
    got_right_thermal_data_ = true;
    time_last_right_thermal_data_ = ros::Time::now();
    got_new_right_thermal_data_ = true;
  }

  {
    std::scoped_lock lock(mutex_right_thermal_data_);
    right_thermal_layout_ = msg->layout;
    //right_thermal_data_ = std::make_unique<std_msgs::Float64 []>(msg->data);
    right_thermal_data_ = msg->data;
  }
  ROS_INFO("Right Message");
}

/* Dynamic reconfigure Callback */
void ThermalStereo::callbackDynamicReconfigure([[maybe_unused]] Config &config, [[maybe_unused]] uint32_t level)
{

  if (!is_initialized_)
    return;

  ROS_INFO(
      "[BallDetection]:"
      "Reconfigure Request: "
      "Focal parameter: %2.2f",
      config.focal_param);
  {
    std::scoped_lock lock(mutex_focal_param_);
    _focal_param_ = config.focal_param;
  }
}

/* callbackTimerCheckSubscribers() method //{ */
void ThermalStereo::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent &te)
{

  if (!is_initialized_)
    return;

  ros::Time time_now = ros::Time::now();

  // Because got_image_, time_last_image_, got_camera_info_ and time_last_camera_info_ are accessed
  // from multiple threads, this mutex is needed to prevent data races.
  // mutex_counters_ is released in the destructor of lock when lock goes out of scope (see below)
  std::scoped_lock lock(mutex_left_thermal_counters_, mutex_right_thermal_counters_);

  /* check whether camera image msgs are coming */
  if (!got_left_thermal_data_)
  {
    ROS_WARN_THROTTLE(1.0, "Not received left thermal data since node launch.");
  }
  else
  {
    if ((time_now - time_last_left_thermal_data_).toSec() > 1.0)
    {
      ROS_WARN_THROTTLE(1.0, "Not received left thermal data msg for %f sec.", (time_now - time_last_left_thermal_data_).toSec());
    }
  }

  if (!got_right_thermal_data_)
  {
    ROS_WARN_THROTTLE(1.0, "Not received rigth thermal data since node launch.");
  }
  else
  {
    if ((time_now - time_last_right_thermal_data_).toSec() > 1.0)
    {
      ROS_WARN_THROTTLE(1.0, "Not received rigth thermal data msg for %f sec.", (time_now - time_last_right_thermal_data_).toSec());
    }
  }
}

std::vector<std::vector<double>> ThermalStereo::getCenterCoords(std::vector<double> *thermal_data)
{
  _Float32 arr[1024];
  _Float32 temperature_matrix[32][32];
  std::vector<std::vector<int>> logical_matrix;
  std::vector<std::vector<int>> cluster_matrix;

  double max_temp = std::max_element(thermal_data->begin(), thermal_data->end())[0];
  int i = 0;

  // set all the remaining numbers
  for (std::vector<double>::const_iterator it = thermal_data->begin(); it != thermal_data->end(); ++it)
  {
    arr[i++] = *it > _start_temperature_ ? *it : 0;
  }

  for (int i = 0; i < 32; i++)
  {
    for (int j = 0; j < 32; j++)
    {
      temperature_matrix[i][j] = arr[32 * i + j] < _heatmap_ratio_ * max_temp ? 0 : arr[32 * i + j];
    }
  }

  // init new coordnates array for further interaction
  std::vector<std::vector<int>> coordinates_matrix;

  logical_matrix = toLogicalArray(temperature_matrix);
  cluster_matrix = toClusterArray(logical_matrix, coordinates_matrix);

  return getMassCenters(temperature_matrix, coordinates_matrix);
}

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

std::vector<double> ThermalStereo::getDistance(std::vector<std::vector<double>> centers_vector_right, std::vector<std::vector<double>> centers_vector_left)
{
  std::vector<double> distanceVector;
  double distorsion = 0;

  int leftIterator = 0;
  int rightIterator = 0;

  while ((leftIterator < centers_vector_left.size()) && (rightIterator < centers_vector_right.size()))
  {
    ROS_INFO("Stucked");

    double centerYLeft = centers_vector_left[leftIterator][0];
    double centerYRight = centers_vector_right[rightIterator][0];

    double centerXLeft = centers_vector_left[leftIterator][1];
    double centerXRight = centers_vector_right[rightIterator][1];

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
        distorsion = abs(centerXLeft - centerXRight);
        distanceVector.push_back((_focal_param_ * _baseline_) / distorsion);
      }

      leftIterator++;
      rightIterator++;
    }
  }

  return distanceVector;
}

std::vector<std::vector<int>> ThermalStereo::toLogicalArray(_Float32 temp_matrix[][32])
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
std::vector<std::vector<int>> ThermalStereo::toClusterArray(std::vector<std::vector<int>> logical_vector, std::vector<std::vector<int>> &coordinates_map)
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

        //ROS_INFO("clusterfind %d", clusterfind);

        if (clusterfind > 0)
        {

          logical_vector[i][j] = clusterfind;
          //ROS_INFO("SEGMENTATION ALERT: %d", coordinates_map.size());
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

std::vector<std::vector<double>> ThermalStereo::getMassCenters(_Float32 temperature_matrix[32][32], std::vector<std::vector<int>> coordinates_vector)
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

void ThermalStereo::main_loop([[maybe_unused]] const ros::TimerEvent &te)
{
  //ROS_INFO("Main_Loop: Start");
  sensor_msgs::Image dbg_msg;

  if (got_new_left_thermal_data_ && got_new_right_thermal_data_)
  {
    std::scoped_lock lock(mutex_left_thermal_data_, mutex_right_thermal_data_);
    //ROS_INFO("Main_Loop: If");
    centers_vector_left_ = getCenterCoords(&left_thermal_data_);
    //ROS_INFO("Main_Loop: Left");
    centers_vector_right_ = getCenterCoords(&right_thermal_data_);
    //ROS_INFO("Main_Loop: Right");

    //            //whatever you'd need that value for (the position is more likely what you're after)
    //            double centerOfMassValue = Temperature_matrix[(int)centersVector.at(0)][(int)centersVector.at(1)] = 249;

    //            cv::Mat image = cv::Mat(32, 32, CV_32FC1, temperature_matrix);
    //            std_msgs::Header header_;
    //            cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_32FC1, image);
    //
    //            img_bridge.toImageMsg(dbg_msg);
    //
    //            // Publish to topic
    //            dbg_pub_true_temp.publish(dbg_msg);

    ROS_INFO("Distance: %f", getDistance(centers_vector_right_, centers_vector_left_)[0]);

    got_new_left_thermal_data_ = false;
    got_new_left_thermal_data_ = false;
  }
  //ROS_INFO("Main_Loop: End");
}

void ThermalStereo::onInit()
{
  got_left_thermal_data_ = false;
  got_right_thermal_data_ = false;
  got_new_left_thermal_data_ = false;
  got_new_right_thermal_data_ = false;

  // get nodelet handle
  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
  // waits for ros to publish clock
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  // Load parameters from .yaml files
  mrs_lib::ParamLoader param_loader(nh_, "ThermalStereo");
  //loading from default.yaml
  param_loader.load_param("rate/check_subscriber", _rate_timer_check_subscribers_);
  //loading from RUN_TYPE.yaml
  param_loader.load_param("camera/thermal/heatmap_ratio", _heatmap_ratio_);
  assert(_heatmap_ratio_ > 0);
  param_loader.load_param("camera/thermal/start_temperature", _start_temperature_);
  assert(_start_temperature_ > 0 && _start_temperature_ < 1000);
  param_loader.load_param("camera/thermal/baseline", _baseline_);
  assert(_baseline_ > 0 && _baseline_ < 10);
  param_loader.load_param("camera/thermal/focal_param", _focal_param_);
  assert(_focal_param_ > 0 && _focal_param_ < 100);
  param_loader.load_param("camera/thermal/init_dist", _init_dist_);
  assert(_init_dist_ >= 0);
  param_loader.load_param("camera/thermal/frequency", _loop_rate_);
  assert(_loop_rate_ > 0);

  //    /* initialize the image transport, needs node handle */
  //    image_transport::ImageTransport it(nh_);

  // Subscribers for temperature array
  thermal_right_subscriber_ = nh_.subscribe("right_termal", 1, &ThermalStereo::tempCallbackRight, this); // Right camera
  thermal_left_subscriber_ = nh_.subscribe("left_termal", 1, &ThermalStereo::tempCallbackLeft, this);    // Left camera

  // Publishers for temperature arrays (to CV Image)
  dbg_left_true_temp_pub_ = nh_.advertise<sensor_msgs::Image>("debug_thermal_left", 10);
  dbg_right_true_temp_pub_ = nh_.advertise<sensor_msgs::Image>("debug_thermal_right", 10);

  //        image_color_subscriber_ = nh_.subscribe("rgb_image", 10, &BallDetection::imageColorCallback, this);
  //        image_depth_subscriber_ = nh_.subscribe("dm_image", 10, &BallDetection::imageDepthCallback, this);
  //camera_info_subscriber_ = nh_.subscribe("rgb_camera_info", 1, &BallDetection::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());

  //        dbg_color_pub_ = it.advertise("debug_color_image", 10);
  //        dbg_depth_pub_ = it.advertise("debug_depth_image", 10);
  m_pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud>("detected_objects_pcl", 10);

  //        rgb_camera_model_.fromCameraInfo(ros::topic::waitForMessage<sensor_msgs::CameraInfo>(nh_.resolveName("rgb_camera_info")));
  //        dm_camera_model_.fromCameraInfo(ros::topic::waitForMessage<sensor_msgs::CameraInfo>(nh_.resolveName("dm_camera_info")));

  // | -------------------- initialize timers ------------------- |
  m_main_loop_timer_ = nh_.createTimer(ros::Rate(_loop_rate_), &ThermalStereo::main_loop, this);
  timer_check_subscribers_ = nh_.createTimer(ros::Rate(_rate_timer_check_subscribers_), &ThermalStereo::callbackTimerCheckSubscribers, this);

  //----------initialization of dynamic reconfigure server-------------
  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfidure_, nh_));
  ReconfigureServer::CallbackType f = boost::bind(&ThermalStereo::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // setting default values of dynamicly reconfigured variables
  {
    std::scoped_lock lock(mutex_focal_param_);
    last_drs_config_.focal_param = _focal_param_;
  }

  reconfigure_server_->updateConfig(last_drs_config_);

  ROS_INFO_THROTTLE(1.0, "[%s]: Node Initialized", m_node_name_.c_str());

  is_initialized_ = true;
}

} // namespace thermal_stereo

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(thermal_stereo::ThermalStereo, nodelet::Nodelet)
