#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <vector>
#include <mutex>

#include "ros_custom_hardware_adapter/Sensor/SensorBase.hpp"
#include "ros_node_utils/RosNodeModuleBase.hpp"
#include "ros_custom_estimator/SensorHandler/SensorHandlerBase.hpp"
using namespace ros_node_utils;

namespace sensor {

class  ParticleFilterSensorHandlerBase : public SensorHandlerBase
{
 public:
  ParticleFilterSensorHandlerBase(ros::NodeHandle* nodeHandle,SensorBase& sensor)
      : SensorHandlerBase(nodeHandle,sensor)
  {

  }

  virtual double probabilityDensity(Eigen::VectorXd x){

  }
  
};
}  // namespace sensor
