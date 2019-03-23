/*
 File name: EstimatorBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 21.03.2019
 */


#pragma once

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>

#include <std_srvs/SetBool.h>
#include <std_msgs/Float64MultiArray.h>

#include "ros_node_base/RosExecuterNodeBase.hpp"

using namespace ros_node_utils;
namespace estimator {
class EstimatorBase : public RosExecuterNodeBase
{
 public:
  EstimatorBase(std::string nodeName)
      : RosExecuterNodeBase(nodeName),
        estimatorRate_(100),
        isSimulation_(true),
        run_(false),
        rate_(),
        dt_(0.0)
  {
  }
  ;
  virtual ~EstimatorBase()
  {
  }
  ;




  virtual void advance(double dt)
  {
  }
  ;

  virtual void execute()
  {
    while (ros::ok()) {
      if (run_) {
        advance(dt_);
      }
      ros::spinOnce();
      rate_->sleep();
    }
  }
  ;

  virtual void readParameters()
  {
    paramRead(this->nodeHandle_,this->namespace_+ "/estimator/simulation", isSimulation_);
    paramRead(this->nodeHandle_,this->namespace_+ "/estimator/rate", estimatorRate_);
    dt_ = 1.0 / estimatorRate_;
    rate_ = new ros::Rate(estimatorRate_);
    paramRead(this->nodeHandle_,this->namespace_+ "/estimator/services/stop/topic", stopServiceName_);
  }
  ;

 protected:


  virtual void initializeServices() override
  {
    stopServices_ = nodeHandle_->advertiseService(this->namespace_ + "/estimator/" + stopServiceName_,
                                                  &EstimatorBase::estimatorStopServiceCallback,
                                                  this);
  }
  ;

  bool estimatorStopServiceCallback(std_srvs::SetBool::Request& request,
                                    std_srvs::SetBool::Response& response)
  {
    run_ = !request.data;
    response.success = true;
    return true;
  }
  ;

  std::string robotName_;

  std::mutex mutex_;

  double estimatorRate_;
  double dt_;


  bool isSimulation_;
  std::string stopServiceName_;

  ros::ServiceServer stopServices_;

  // Flag for running
  bool run_;

};
}  // namespace estimator
