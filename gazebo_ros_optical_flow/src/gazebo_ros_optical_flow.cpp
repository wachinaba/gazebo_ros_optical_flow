#include "gazebo_ros_optical_flow/gazebo_ros_optical_flow.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <gazebo_ros_optical_flow_msgs/OpticalFlow.h>

#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosOpticalFlowPlugin)

GazeboRosOpticalFlowPlugin::GazeboRosOpticalFlowPlugin() : SensorPlugin(), width(0), height(0), depth(0)
{
}

GazeboRosOpticalFlowPlugin::~GazeboRosOpticalFlowPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
  ROS_DEBUG_STREAM_NAMED("camera", "UnLoaded");
}

void GazeboRosOpticalFlowPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "GazeboRosOpticalFlow plugin requires a CameraSensor.\n";
  }

  if (!this->parentSensor)
  {
    gzerr << "GazeboRosOpticalFlow plugin not attached to a camera sensor\n";
    return;
  }

  this->world = physics::get_world(this->parentSensor->WorldName());

  this->camera = this->parentSensor->Camera();
  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();
  hfov_ = float(this->camera->HFOV().Radian());
  first_frame_time_ = this->camera->LastRenderWallTime().Double();
  const string scopedName = _sensor->ParentName();

  focal_length_ = (this->width / 2) / tan(hfov_ / 2);

  if (this->width != 64 || this->height != 64)
  {
    gzerr << "[gazebo_optical_flow_plugin] Incorrect image size, must by 64 x 64.\n";
  }

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_optical_flow_plugin] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("outputRate"))
  {
    output_rate_ = _sdf->GetElement("outputRate")->Get<int>();
  }
  else
  {
    output_rate_ = DEFAULT_RATE;
    gzwarn << "[gazebo_optical_flow_plugin] Using default output rate " << output_rate_ << ".";
  }

  if (_sdf->HasElement("searchSize"))
  {
    search_size_ = _sdf->GetElement("searchSize")->Get<int>();
  }
  else
  {
    search_size_ = DEFAULT_SEARCH_SIZE;
    gzwarn << "[gazebo_optical_flow_plugin] Using default search size " << search_size_ << ".";
  }

  if (_sdf->HasElement("flowFeatureThreshold"))
  {
    flow_feature_threshold_ = _sdf->GetElement("flowFeatureThreshold")->Get<int>();
  }
  else
  {
    flow_feature_threshold_ = DEFAULT_FLOW_FEATURE_THRESHOLD;
    gzwarn << "[gazebo_optical_flow_plugin] Using default flow feature threshold " << flow_feature_threshold_ << ".";
  }

  if (_sdf->HasElement("flowValueThreshold"))
  {
    flow_value_threshold_ = _sdf->GetElement("flowValueThreshold")->Get<int>();
  }
  else
  {
    flow_value_threshold_ = DEFAULT_FLOW_VALUE_THRESHOLD;
    gzwarn << "[gazebo_optical_flow_plugin] Using default flow value threshold " << flow_value_threshold_ << ".";
  }

  if (_sdf->HasElement("hasGyro"))
    has_gyro_ = _sdf->GetElement("hasGyro")->Get<bool>();
  else
    has_gyro_ = HAS_GYRO;

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (has_gyro_)
  {
    if (_sdf->HasElement("hasGyro"))
      gyro_sub_topic_ = _sdf->GetElement("gyroTopic")->Get<std::string>();
    else
      gyro_sub_topic_ = "/optical_flow/imu";

    string topicName = "~/" + _sensor->ParentName() + gyro_sub_topic_;
    boost::replace_all(topicName, "::", "/");
    imuSub_ = node_handle_->Subscribe(topicName, &GazeboRosOpticalFlowPlugin::ImuCallback, this);
  }

  if (_sdf->HasElement("topicName"))
    ros_topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  else
    ros_topic_name_ = "/optical_flow";

  if (_sdf->HasElement("deltaTopicName"))
    ros_delta_topic_name_ = _sdf->GetElement("deltaTopicName")->Get<std::string>();
  else
    ros_delta_topic_name_ = "/optical_flow_delta";

  string topicName = "~/" + scopedName + "/optical_flow";
  boost::replace_all(topicName, "::", "/");

  this->newFrameConnection = this->camera->ConnectNewImageFrame(boost::bind(
      &GazeboRosOpticalFlowPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

  this->parentSensor->SetActive(true);

  // init flow
  // optical_flow_ = new OpticalFlowOpenCV(focal_length_, focal_length_, output_rate_);
  optical_flow_ = new OpticalFlowPX4(focal_length_, focal_length_, output_rate_, this->width, this->height,
                                     search_size_, flow_feature_threshold_, flow_value_threshold_);

  nh_ = ros::NodeHandle("/");

  // init ros msg publisher
  opticalFlowRosPub_ = nh_.advertise<mavros_msgs::OpticalFlowRad>(ros_topic_name_, 1);
  opticalFlowDeltaPub_ = nh_.advertise<optical_flow_msgs::OpticalFlowDelta>(ros_delta_topic_name_, 1);
}

void GazeboRosOpticalFlowPlugin::OnNewFrame(const unsigned char* _image, unsigned int _width, unsigned int _height,
                                            unsigned int _depth, const std::string& _format)
{
  _image = this->camera->ImageData(0);
  double frame_time = this->world->SimTime().Double();

  // debug: print frame time
  printf("frame time: %f\n", frame_time);

  frame_time_us_ = (frame_time - first_frame_time_) * 1e6;  // since start

  float flow_x_ang = 0.0f;
  float flow_y_ang = 0.0f;
  float flow_delta_x = 0;
  float flow_delta_y = 0;

  // calculate angular flow
  int quality = optical_flow_->calcFlowWithDelta((uchar*)_image, frame_time_us_, dt_us_, flow_x_ang, flow_y_ang,
                                                 flow_delta_x, flow_delta_y);

  opticalFlowRosMsg_.header.stamp = ros::Time::now();
  opticalFlowRosMsg_.integration_time_us = dt_us_;
  opticalFlowRosMsg_.integrated_x = flow_x_ang;
  opticalFlowRosMsg_.integrated_y = flow_y_ang;
  if (has_gyro_)
  {
    opticalFlowRosMsg_.integrated_xgyro = opticalFlow_rate.X();
    opticalFlowRosMsg_.integrated_ygyro = opticalFlow_rate.Y();
    opticalFlowRosMsg_.integrated_zgyro = opticalFlow_rate.Z();
    opticalFlow_rate.Set();
  }
  else
  {
    opticalFlowRosMsg_.integrated_xgyro = NAN;
    opticalFlowRosMsg_.integrated_ygyro = NAN;
    opticalFlowRosMsg_.integrated_zgyro = NAN;
  }
  opticalFlowRosMsg_.temperature = 20.0f;
  opticalFlowRosMsg_.quality = quality;
  opticalFlowRosMsg_.time_delta_distance_us = 0;
  opticalFlowRosMsg_.distance = 0.0f;

  opticalFlowDeltaMsg_.header.stamp = ros::Time::now();
  opticalFlowDeltaMsg_.integration_time_us = dt_us_;
  opticalFlowDeltaMsg_.delta_px = flow_delta_x;
  opticalFlowDeltaMsg_.delta_py = flow_delta_y;
  opticalFlowDeltaMsg_.surface_quality = quality;

  opticalFlowRosPub_.publish(opticalFlowRosMsg_);
  opticalFlowDeltaPub_.publish(opticalFlowDeltaMsg_);
}

void GazeboRosOpticalFlowPlugin::ImuCallback(ConstIMUPtr& _imu)
{
  // accumulate gyro measurements that are needed for the optical flow message
  common::Time now = world->SimTime();

  uint32_t now_us = now.Double() * 1e6;
  ignition::math::Vector3d px4flow_gyro = ignition::math::Vector3d(
      _imu->angular_velocity().x(), _imu->angular_velocity().y(), _imu->angular_velocity().z());

  static uint32_t last_dt_us = now_us;
  uint32_t dt_us = now_us - last_dt_us;

  if (dt_us > 1000)
  {
    opticalFlow_rate += px4flow_gyro * (dt_us / 1000000.0f);
    last_dt_us = now_us;
  }
}
}  // namespace gazebo
