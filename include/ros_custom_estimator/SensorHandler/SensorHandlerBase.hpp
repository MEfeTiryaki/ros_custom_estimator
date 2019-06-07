#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <vector>
#include <mutex>

#include "ros_node_utils/RosNodeModuleBase.hpp"
using namespace ros_node_utils;

namespace sensor {

class SensorHandlerBase : public RosNodeModuleBase
{
 public:
  SensorHandlerBase(ros::NodeHandle* nodeHandle)
      : RosNodeModuleBase(nodeHandle),
        dt_(0.0),
        isMeasurementUpdated_(false),
        hardwareAdapterFrame_()
  {

  }

  virtual ~SensorHandlerBase()
  {
  }

  virtual void create(hardware_adapter::HardwareAdapterFrameBase* hardwareAdapterFrame){
   hardwareAdapterFrame_ = hardwareAdapterFrame ;
  }

  virtual void initialize()override {

  }


  /*! SensorHandler get data here
   *
   */
  virtual void advance(){

    isMeasurementUpdated_ = false;
  }

  void reset()
  {
    isMeasurementUpdated_ = false;
  }

  bool isUpdated()
  {
    return isMeasurementUpdated_;
  }

  virtual Eigen::VectorXd getTransition(Eigen::VectorXd x, Eigen::VectorXd u)
  {
    return Eigen::VectorXd();
  }

  virtual Eigen::VectorXd getObservation(Eigen::VectorXd x)
  {
    return Eigen::VectorXd();
  }


  Eigen::MatrixXd getObservationJacobian()
  {
    return H_;
  }

  Eigen::MatrixXd getObservationNoiseCovariance()
  {
    return R_;
  }

  void setDeltaT(double dt)
  {
    dt_ = dt;
  }


 protected:

  bool isMeasurementUpdated_;
  double dt_;

  hardware_adapter::HardwareAdapterFrameBase* hardwareAdapterFrame_;

  Eigen::VectorXd sensorData_;
  double lastSensorDataTimeStamp_;

  Eigen::VectorXd z_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;

};
}  // namespace sensor
