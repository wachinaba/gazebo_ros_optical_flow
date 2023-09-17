#ifndef GAZEBO_ROS_OPTICAL_FLOW_SENSOR_HH
#define GAZEBO_ROS_OPTICAL_FLOW_SENSOR_HH

#include <string>

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ignition/math.hh>

#include "flow_opencv.hpp"
#include "flow_px4.hpp"

#include "mavros_msgs/OpticalFlowRad.h"
#include <optical_flow_msgs/OpticalFlowDelta.h>

using namespace cv;
using namespace std;

#define DEFAULT_RATE 20
#define HAS_GYRO true

namespace gazebo
{
class GAZEBO_VISIBLE GazeboRosOpticalFlowPlugin : public SensorPlugin
{
public:
  GazeboRosOpticalFlowPlugin();
  ~GazeboRosOpticalFlowPlugin();

  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  virtual void OnNewFrame(const unsigned char* _image, unsigned int _width, unsigned int _height, unsigned int _depth,
                          const std::string& _format);
  void ImuCallback(ConstIMUPtr& _imu);

protected:
  unsigned int width, height, depth;
  std::string format;
  sensors::CameraSensorPtr parentSensor;
  rendering::CameraPtr camera;
  physics::WorldPtr world;

private:
  event::ConnectionPtr newFrameConnection;
  transport::PublisherPtr opticalFlow_pub_;
  transport::NodePtr node_handle_;
  transport::SubscriberPtr imuSub_;
  ignition::math::Vector3d opticalFlow_rate;
  std::string namespace_;
  std::string gyro_sub_topic_;
  // OpticalFlowOpenCV *optical_flow_;
  OpticalFlowPX4* optical_flow_;

  // ros OpticalFlow message node handler and publishers
  ros::NodeHandle nh_;
  ros::Publisher opticalFlowRosPub_;
  ros::Publisher opticalFlowDeltaPub_;

  std::string ros_topic_name_;
  std::string ros_delta_topic_name_;

  mavros_msgs::OpticalFlowRad opticalFlowRosMsg_;
  optical_flow_msgs::OpticalFlowDelta opticalFlowDeltaMsg_;

  float hfov_;
  int dt_us_;
  int output_rate_;
  float focal_length_;
  double first_frame_time_;
  uint32_t frame_time_us_;
  bool has_gyro_;

  int search_size_;
  int flow_feature_threshold_;
  int flow_value_threshold_;
};
}  // namespace gazebo

#endif
