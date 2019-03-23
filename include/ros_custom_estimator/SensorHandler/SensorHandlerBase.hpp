#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <vector>
#include <mutex>

#include "ros_node_base/RosNodeModuleBase.hpp"
using namespace ros_node_utils;

namespace sensor {

class SensorHandlerBase : public RosNodeModuleBase
{
 public:
  SensorHandlerBase(ros::NodeHandle* nodeHandle, std::mutex &mutex, int id)
      : RosNodeModuleBase(nodeHandle),
        mutex_(mutex),
        id_(id),
        dt_(0.0),
        isMeasurement_(false)
  {

  }
  ;

  virtual ~SensorHandlerBase()
  {
  }
  ;


  void reset()
  {
    isMeasurement_ = false;
  }
  ;

  bool isUpdated()
  {
    return isMeasurement_;
  }
  ;

  virtual Eigen::VectorXd getData(Eigen::VectorXd x, Eigen::VectorXd u)
  {
    return Eigen::VectorXd();
  }
  ;

  virtual Eigen::VectorXd geth(Eigen::VectorXd x)
  {
    return Eigen::VectorXd();
  }
  ;

  virtual double probabilityDensity(Eigen::VectorXd w){
    return 0.0;
  };


  int getId()
  {
    return id_;
  }
  ;

  Eigen::MatrixXd getH()
  {
    return H_;
  }
  ;
  Eigen::MatrixXd getR()
  {
    return R_;
  }
  ;
  void setDeltaT(double dt)
  {
    dt_ = dt;
  }
  ;

  void setSubscriberName(std::string name)
  {
    subscriberName_ = name;
  }
  ;

 protected:

  std::mutex &mutex_;

  int id_;

  bool isMeasurement_;
  double dt_;

  std::string subscriberName_;

  Eigen::VectorXd z_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;

};
}  // namespace sensor
