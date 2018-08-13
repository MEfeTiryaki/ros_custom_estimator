#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <vector>
#include <mutex>

namespace sensor {

class SensorHandlerBase
{
 public:
  SensorHandlerBase(std::mutex &mutex, int id)
      : mutex_(mutex),
        id_(id),
        dt_(0.0),
        isMeasurement_(false)
  {
    ns_ = ros::this_node::getNamespace();
    ns_.erase(0, 1);
  }
  ;

  virtual ~SensorHandlerBase()
  {
  }
  ;

  virtual void create()
  {
  }
  ;

  virtual void readParameters()
  {
  }
  ;
  virtual void initilize(ros::NodeHandle* nodeHandle)
  {
    nodeHandle_ = nodeHandle;
  }

  virtual void initilizePublishers()
  {
  }
  ;
  virtual void initilizeSubscribers()
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
  ros::NodeHandle* nodeHandle_;
  std::string ns_;
  std::mutex &mutex_;
  int id_;
  bool isMeasurement_;
  double dt_;

  ros::Publisher publisher;

  ros::Subscriber subscriber_;
  std::string subscriberName_;

  Eigen::VectorXd z_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;

};
}  // namespace sensor
