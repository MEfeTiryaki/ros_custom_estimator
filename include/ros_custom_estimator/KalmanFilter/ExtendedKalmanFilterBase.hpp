#pragma once

#include "ros_custom_estimator/KalmanFilter/KalmanFilterBase.hpp"

namespace estimator {
class ExtendedKalmanFilterBase : public KalmanFilterBase
{
 public:
  ExtendedKalmanFilterBase(ros::NodeHandle* nodeHandle,
      hardware_adapter::HardwareBase& hardwareAdapterFrame,
      robot::RobotContainerBase& robot)
  : KalmanFilterBase(nodeHandle, hardwareAdapterFrame, robot)
  {
  }


  ~ExtendedKalmanFilterBase()
  {
  }
  
};
}
