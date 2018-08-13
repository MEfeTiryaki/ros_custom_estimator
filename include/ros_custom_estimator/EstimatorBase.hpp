/*
 File name: EstimatorBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 19.06.2018
 */
#pragma once

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>

#include <std_srvs/SetBool.h>
namespace estimator {
class EstimatorBase
{
 public:
  EstimatorBase()
      : estimatorRate_(100),
        isSimulation_(true),
        run_(false),
        rate_(),
        dt_(0.0)
  {
    nodeName_ = "estimator";
    int argc = 0;
    char **argv = {};
    ros::init(argc,argv, nodeName_);
    ns_ = ros::this_node::getNamespace();
    ns_.erase(0, 1);

    //nodeHandle_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle ("~"));
    this->nodeHandle_ = new ros::NodeHandle("~");
    this->nodeName_ = ros::this_node::getName();
  }
  ;
  virtual ~EstimatorBase()
  {
  }
  ;

  virtual void create()
  {
  }
  ;

  virtual void initilize()
  {

    readParameters();

    initilizePublishers();
    initilizeSubscribers();
    initilizeServices();
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
    nodeHandle_->getParam(ns_ + "/estimator/simulation", isSimulation_);
    nodeHandle_->getParam(ns_ + "/estimator/rate", estimatorRate_);
    dt_ = 1.0 / estimatorRate_;
    rate_ = new ros::Rate(estimatorRate_);
    nodeHandle_->getParam(ns_ + "/estimator/services/stop/topic", stopServiceName_);
  }
  ;

 protected:
  virtual void initilizeSubscribers()
  {

  }
  ;

  virtual void initilizePublishers()
  {
  }
  ;

  virtual void initilizeServices()
  {
    stopServices_ = nodeHandle_->advertiseService(ns_ + "/estimator/" + stopServiceName_,
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

  std::string ns_;
  std::string nodeName_;
  std::string robotName_;
  ros::NodeHandle* nodeHandle_;
  ros::Rate* rate_;
  std::mutex mutex_;
  double estimatorRate_;
  double dt_;


  bool isSimulation_;
  std::string stopServiceName_;

  std::vector<ros::Publisher> publishers_;
  std::vector<ros::Subscriber> subscribers_;
  ros::ServiceServer stopServices_;
  // Flag for running
  bool run_;

};
}  // namespace estimator
