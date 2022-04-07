/*
 File name: EstimatorBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 21.03.2019
 */

#pragma once

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <math.h>
#include <memory>
#include <mutex>
#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>

#include "ros_node_utils/RosNodeModuleBase.hpp"
#include "ros_custom_estimator/EstimatorBase.hpp"

using namespace ros_node_utils;
namespace estimator {
class EstimatorManager  : public RosNodeModuleBase {
public:
  EstimatorManager(ros::NodeHandle *nodeHandle)
      : RosNodeModuleBase(nodeHandle)
      {}

  virtual ~EstimatorManager() {}

  virtual void create() {
    RosNodeModuleBase::create();
    for (int i = 0; i < estimatorNameList_.size(); i++) {
      estimators_[i]->create();
    }

    if (estimatorNameList_.size() > 0) {
      estimatorName_ = estimatorNameList_[0];
      estimatorId_ = 0;
    } else {
      estimatorName_ = "empty";
      estimatorId_ = -1;
    }

  }

  virtual void readParameters() {
    RosNodeModuleBase::readParameters();
    for (int i = 0; i < estimatorNameList_.size(); i++) {
      estimators_[i]->readParameters();
    }

    std::string name;
    paramRead(getNodeHandle(),
              "/" + this->namespace_ + "/estimator/estimator_name",
              name);

    for (int i = 0; i < estimatorNameList_.size(); i++) {
      if (estimatorNameList_[i] == name) {
        estimatorName_ = name;
        estimatorId_ = i;
        break;
      }
    }


    CONFIRM("Estimator is " + estimatorName_);
  }

  virtual void initialize() override {
    RosNodeModuleBase::initialize();
    for (int i = 0; i < estimatorNameList_.size(); i++) {
      estimators_[i]->initialize();
    }

  }

  void initializePublishers() {
    for (int i = 0; i < estimatorNameList_.size(); i++) {
      estimators_[i]->initializePublishers();
    }
  }

  void initializeSubscribers() {
    trajectorySubscriber_ = getNodeHandle()->subscribe(
        "/" + this->namespace_ + "/estimator/estimator_manager", 1,
        &EstimatorManager::trajectoryManagerCallback, this);
    for (int i = 0; i < estimatorNameList_.size(); i++) {
      estimators_[i]->initializeSubscribers();
    }
  }

  void initializeServices() {
    for (int i = 0; i < estimatorNameList_.size(); i++) {
      estimators_[i]->initializeServices();
    }
  }


  virtual void shutdown() override {
    std::lock_guard<std::mutex> lock(*shutdownMutex_);
    RosNodeModuleBase::shutdown();

  }



protected:
  ros::Subscriber estimationSubscriber_;
  std::vector<std::unique_ptr<EstimatorBase>> estimators_;
  std::vector<std::string> estimatorNameList_;
  std::string estimatorName_;
  int estimatorId_;


};
} // namespace estimator
