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
  EstimatorBase(ros::NodeHandle* nodeHandle,
                hardware_adapter::HardwareAdapterFrameBase& hardwareAdapterFrame,
                robot::RobotContainerBase& robot)
      : RosNodeModuleBase(nodeHandle),
        hardwareAdapterFrame_(hardwareAdapterFrame),
        robot_(robot),
        estimatorRate_(100),
        isSimulation_(true),
        run_(false),
        dt_(0.0),
        estimatorMutex_(new std::mutex()),
        estimatorThread_(),
        active_(true)
  {
  }

  virtual ~EstimatorBase()
  {
  }

  virtual void create()
  {
    RosNodeModuleBase::create();
    estimatorRate_ = 100;
    isSimulation_ = true;
    run_ = false;
    dt_ = 0.0;
  }

  virtual void readParameters()
  {
    RosNodeModuleBase::readParameters();
    paramRead(this->nodeHandle_, "/simulation", isSimulation_);
    paramRead(this->nodeHandle_ , "/" + this->namespace_ + "/estimator/rate", estimatorRate_);
    paramRead(this->nodeHandle_ , "/" + this->namespace_ + "/estimator/services/stop/topic",
              stopServiceName_);
  }

  virtual void initialize() override
  {
    RosNodeModuleBase::initialize();
    dt_ = 1.0 / estimatorRate_;
    rate_ = new ros::Rate(estimatorRate_);

    connect();
  }

  virtual void shutdown() override
  {
    std::lock_guard<std::mutex> lock(*shutdownMutex_);
    RosNodeModuleBase::shutdown();
    disconnect();
  }

  virtual void advance(double dt)
  {
  }

  virtual void execute()
  {
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(*shutdownMutex_);
        if (!isTerminationStarted()) {
          if (run_) {
            advance(dt_);
          }
        } else {
          break;
        }
      }
      this->rate_->sleep();
    }
    terminate();
  }

  virtual void connect()
  {
    estimatorThread_ = new boost::thread(boost::bind(&EstimatorBase::execute, this));
  }

  virtual void disconnect()
  {
    estimatorThread_->detach();
  }

  std::mutex* getEstimatorMutex()
  {
    return estimatorMutex_;
  }

  void setRun(bool run)
  {
    run_ = run;
  }

  void setActive(bool active)
  {
    active_ = active;
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
  bool active_;

  hardware_adapter::HardwareAdapterFrameBase& hardwareAdapterFrame_;
  robot::RobotContainerBase& robot_;
};
}  // namespace estimator
