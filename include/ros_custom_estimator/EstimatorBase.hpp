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

#include "ros_node_utils/RosNodeModuleBase.hpp"
#include "ros_custom_hardware_adapter/HardwareAdapterFrameBase.hpp"
#include "robot_container/RobotContainerBase.hpp"

using namespace ros_node_utils;
namespace estimator {
class EstimatorBase : public RosNodeModuleBase
{
 public:
  EstimatorBase(ros::NodeHandle* nodeHandle)
      : RosNodeModuleBase(nodeHandle),
        estimatorRate_(100),
        isSimulation_(true),
        run_(false),
        dt_(0.0),
        estimatorMutex_(),
        hardwareAdapterFrame_(),
        estimatorThread_(),
        robot_()
  {
  }

  virtual ~EstimatorBase()
  {
  }

  virtual void create(hardware_adapter::HardwareAdapterFrameBase* hardwareAdapterFrame,
                      robot::RobotContainerBase* robot)
  {
    hardwareAdapterFrame_ = hardwareAdapterFrame;
    robot_ = robot;
  }

  virtual void readParameters()
  {
    paramRead(this->nodeHandle_, "/simulation", isSimulation_);
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/rate", estimatorRate_);
    dt_ = 1.0 / estimatorRate_;
    rate_ = new ros::Rate(estimatorRate_);
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/services/stop/topic",
              stopServiceName_);
  }

  virtual void initialize() override
  {
    connectEstimator();

    ERROR("[EstimatorBase] : is initialize. ");
  }
  ;

  virtual void initializeServices() override
  {
    stopServices_ = nodeHandle_->advertiseService(
        this->namespace_ + "/estimator/" + stopServiceName_,
        &EstimatorBase::estimatorStopServiceCallback, this);
  }

  virtual void advance(double dt)
  {

    //sensor_->advance(dt);
  }

  virtual void execute()
  {
    while (ros::ok()) {
      if (run_) {
        advance(dt_);
      }
      ros::spinOnce();
      rate_->sleep();
      //WARNING("[Estimator] : " + std::to_string(ros::Time::now().toNSec()/1000000.0));
    }
  }

  virtual void connectEstimator()
  {
    estimatorThread_ = new boost::thread(boost::bind(&EstimatorBase::execute, this));
  }

  std::mutex* getEstimatorMutex()
  {
    return estimatorMutex_;
  }

  void setRun(bool run)
  {
    run_ = run;
  }

 protected:

  bool estimatorStopServiceCallback(std_srvs::SetBool::Request& request,
                                    std_srvs::SetBool::Response& response)
  {
    run_ = !request.data;
    response.success = true;
    return true;
  }

  std::string robotName_;

  std::mutex* estimatorMutex_;

  boost::thread* estimatorThread_;

  double estimatorRate_;
  double dt_;

  bool isSimulation_;
  std::string stopServiceName_;

  ros::ServiceServer stopServices_;

  // Flag for running
  bool run_;

  hardware_adapter::HardwareAdapterFrameBase* hardwareAdapterFrame_;

  robot::RobotContainerBase* robot_;
};
}  // namespace estimator
