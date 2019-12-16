#pragma once

#include "ros_custom_estimator/KalmanFilter/KalmanFilterBase.hpp"

namespace estimator {
class ExtendedKalmanFilterBase : public KalmanFilterBase
{
 public:
  ExtendedKalmanFilterBase(ros::NodeHandle* nodeHandle,
      SensorBase &sensor, ActuatorBase &actuator,
      robot::RobotContainerBase& robot)
  : KalmanFilterBase(nodeHandle, sensor, actuator,, robot)
  {
  }


  ~ExtendedKalmanFilterBase()
  {
  }

};
}
