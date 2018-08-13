#pragma once

#include "ros_custom_estimator/EstimatorBase.hpp"
#include "ros_custom_estimator/SensorHandler/SensorHandlerBase.hpp"
#include "ros_custom_controller_base/State.h"
#include "ros_custom_controller_base/Input.h"
#include <ctime>

namespace estimator {
class KalmanFilterBase : public EstimatorBase
{
 public:
  KalmanFilterBase():
    EstimatorBase(),
    statePublisherQueueSize_(0),
    publishTime_(true)
  {
    time_start_ = clock();
    time_start_ros_ = ros::Time::now().toSec();
  };

  //~KalmanFilterBase();

  virtual void readParameters() override{
    for (auto s : sensors_) {
      s->readParameters();
      if (nodeHandle_->hasParam(
          ns_ + "/estimator/subscribers/sensorSubscriber_" + std::to_string(s->getId()) + "/topic")) {
        std::string name;
        nodeHandle_->getParam(
            ns_ + "/estimator/subscribers/sensorSubscriber_" + std::to_string(s->getId()) + "/topic",
            name);
        s->setSubscriberName(name);
      }
    }
    // SUBSCRIBERS
    if (nodeHandle_->hasParam(ns_ + "/estimator/subscribers/inputSubscriber/topic")) {
      nodeHandle_->getParam(ns_ + "/estimator/subscribers/inputSubscriber/topic",
                            inputSubscriberName_);
    }

    if (nodeHandle_->hasParam(ns_ + "/estimator/subscribers/initilizerSubscriber/topic")) {
      nodeHandle_->getParam(ns_ + "/estimator/subscribers/initilizerSubscriber/topic",
                            initilizerSubscriberName_);
    }
    if (nodeHandle_->hasParam(ns_ + "/estimator/subscribers/initilizerSubscriber/queue_size")) {
      nodeHandle_->getParam(ns_ + "/estimator/subscribers/initilizerSubscriber/queue_size",
                            initilizerSubscriberName_);
    }

    // PUBLISHERS
    if (nodeHandle_->hasParam(ns_ + "/estimator/publishers/estimator/topic")) {
      nodeHandle_->getParam(ns_ + "/estimator/publishers/estimator/topic", statePublisherName_);
    }
    if (nodeHandle_->hasParam(ns_ + "/estimator/publishers/estimator/queue_size")) {
      nodeHandle_->getParam(ns_ + "/estimator/publishers/estimator/queue_size",
                            statePublisherQueueSize_);
    }
    // PARAMETERS
    // X_O
    double* ptr;
    if (nodeHandle_->hasParam(ns_ + "/estimator/estimatorParameters/X")) {
      std::vector<double> value;
      nodeHandle_->getParam(ns_ + "/estimator/estimatorParameters/X", value);
      ptr = &value[0];
      x_0_ << Eigen::Map<Eigen::VectorXd>(ptr, n_);
      x_m_ << Eigen::Map<Eigen::VectorXd>(ptr, n_);
    }

    // P
    if (nodeHandle_->hasParam(ns_ + "/estimator/estimatorParameters/P")) {
      std::vector<double> value;
      nodeHandle_->getParam(ns_ + "/estimator/estimatorParameters/P", value);
      ptr = &value[0];
      P_init_.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, n_);
      P_m_.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, n_);
    }
    if (nodeHandle_->hasParam(ns_ + "/estimator/estimatorParameters/Q")) {
      std::vector<double> value;
      nodeHandle_->getParam(ns_ + "/estimator/estimatorParameters/Q", value);
      ptr = &value[0];
      Q_.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, n_);

    }
    EstimatorBase::readParameters();
  };

  virtual void create() override
  {
    x_0_ = Eigen::VectorXd::Zero(n_);
    x_m_ = Eigen::VectorXd::Zero(n_);
    P_m_ = Eigen::MatrixXd::Zero(n_, n_);
    P_init_ = Eigen::MatrixXd::Zero(n_, n_);
    Q_ = Eigen::MatrixXd::Zero(n_, n_);
    u_ = Eigen::VectorXd::Zero(m_);
    z_ = Eigen::VectorXd::Zero(n_);

    sensors_ = std::vector<sensor::SensorHandlerBase*>();
    addSensors();
    for (auto& s : sensors_) {
      s->create();
    }
    state_ = ros_custom_controller_base::State();
    publisherTimer_ = nodeHandle_->createTimer(ros::Duration(0.02),
                                               &KalmanFilterBase::publisherTimerCallback, this);
  };

  virtual void initilize() override
  {
    x_p_ = Eigen::VectorXd::Zero(n_);
    P_p_ = Eigen::MatrixXd::Zero(n_, n_);
    u_ = Eigen::VectorXd::Zero(m_);
    z_ = Eigen::VectorXd();
    for (auto& s : sensors_) {
      s->initilize(nodeHandle_);
    }
    EstimatorBase::initilize();
  };

  virtual void initilizeSubscribers() override
  {
    for (auto& s : sensors_) {
      s->initilizeSubscribers();
    }
    inputSubcriber_ = nodeHandle_->subscribe(ns_ + "/" + inputSubscriberName_, 10,
                                             &KalmanFilterBase::inputSubscriberCallback, this);

    initilizerSubcriber_ = nodeHandle_->subscribe(ns_ + "/" + initilizerSubscriberName_,
                                                  initilizerSubscriberQueueSize_,
                                                  &KalmanFilterBase::initilizerSubscriberCallback,
                                                  this);

    EstimatorBase::initilizeSubscribers();

  };

  virtual void initilizePublishers() override
  {
    for (auto& s : sensors_) {
      s->initilizePublishers();
    }
    statePublisher_ = nodeHandle_->advertise<ros_custom_controller_base::State>(
        ns_ + "/" + statePublisherName_, statePublisherQueueSize_);
  };

  virtual void advance(double dt) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Sets the self.z_
    getMeasurements();
    // Prediction Step
    pStep();
    // Measurement Step
    mStep();
    // Publish the state
    publish();
    // Set Measurement and Input flags False
    for (auto& s : sensors_) {
      s->reset();
    }
    time_stop_ = clock();
    time_stop_ros_ = ros::Time::now().toSec();
    //std::cout << "dt : " <<dt_<<" sec" << std::endl;
    //std::cout << "Time: " << (time_stop_-time_start_)/double(CLOCKS_PER_SEC) << " sec "<< std::endl;
    //std::cout << "Rime: " << time_stop_ros_-time_start_ros_<<" sec" <<std::endl;
    //std::cout << "_____________" ;

    time_start_ = clock();
    time_start_ros_ = ros::Time::now().toSec();
  };

  virtual void addSensors()
  {
  }
  ;

  virtual void pStep()
  {
  }
  ;

  virtual void mStep()
  {
  }
  ;

  virtual void getMeasurements()
  {
    z_ = Eigen::VectorXd();
    Eigen::VectorXd dummy = Eigen::VectorXd();
    int size_z;
    for (auto& sensor : sensors_) {
      if (sensor->isUpdated()) {
        dummy = sensor->getData(x_m_, u_);
        size_z = z_.size();
        z_.conservativeResize(size_z + dummy.size());
        z_.segment(size_z, dummy.size()) = dummy;
      }
    }
  };

  virtual void publish()
  {
    state_.header.stamp = ros::Time::now();
    state_.state.clear();
    for (int i = 0; i < x_m_.size(); i++) {
      state_.state.push_back(x_m_(i));
    }
    statePublisher_.publish(state_);
  };

  virtual void inputSubscriberCallback(ros_custom_controller_base::Input msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < msg.input.size(); i++) {
      u_[i] = msg.input[i];
    }
  };

  virtual void initilizerSubscriberCallback(ros_custom_controller_base::State msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    //std::cerr << this->ns_ << " init subscriber!!" << std::endl;
    for (int i = 0; i < msg.state.size(); i++) {
      x_m_[i] = msg.state[i];
    }
    //std::cerr<< x_m_.transpose() << std::endl;
  };


  void publisherTimerCallback(const ros::TimerEvent& event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishTime_ = true;
  };

 protected:
  int n_;
  int m_;


  std::vector<sensor::SensorHandlerBase*> sensors_;
  ros_custom_controller_base::State state_;

  ros::Publisher statePublisher_;
  std::string statePublisherName_;
  int statePublisherQueueSize_;

  ros::Subscriber inputSubcriber_;
  std::string inputSubscriberName_;

  ros::Subscriber initilizerSubcriber_;
  std::string initilizerSubscriberName_;
  int initilizerSubscriberQueueSize_;

  Eigen::VectorXd x_p_;
  Eigen::VectorXd x_m_;
  Eigen::VectorXd x_0_;
  Eigen::MatrixXd P_p_;
  Eigen::MatrixXd P_m_;
  Eigen::MatrixXd P_init_;
  Eigen::MatrixXd Q_;

  Eigen::VectorXd u_;
  Eigen::VectorXd z_;

  ros::Timer publisherTimer_;
  bool publishTime_;

  double time_start_;
  double time_stop_;
  double time_start_ros_;
  double time_stop_ros_;
};
}
