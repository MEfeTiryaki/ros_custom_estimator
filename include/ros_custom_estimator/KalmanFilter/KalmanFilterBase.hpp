#pragma once

#include "ros_custom_estimator/EstimatorBase.hpp"
#include "ros_custom_estimator/SensorHandler/SensorHandlerBase.hpp"
#include <ctime>

namespace estimator {
class KalmanFilterBase : public EstimatorBase
{
 public:
  KalmanFilterBase(std::string nodeName)
      : EstimatorBase(nodeName),
        statePublisherQueueSize_(0),
        publishTime_(true),
        m_(0),
        n_(0),
        initializerSubscriberQueueSize_(1)
  {
    time_start_ = clock();
    time_start_ros_ = ros::Time::now().toSec();
  }
  ;

  //~KalmanFilterBase();

  virtual void readParameters() override
  {

    for (auto s : sensors_) {
      s->readParameters();
      std::string name;
      paramRead(
          this->nodeHandle_,
          this->namespace_ + "/estimator/subscribers/sensorSubscriber_" + std::to_string(s->getId())
              + "/topic",
          name);
      s->setSubscriberName(name);
    }
    // SUBSCRIBERS
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/subscribers/inputSubscriber/topic",
              inputSubscriberName_);

    paramRead(this->nodeHandle_,
              this->namespace_ + "/estimator/subscribers/initializerSubscriber/topic",
              initializerSubscriberName_);

    paramRead(this->nodeHandle_,
              this->namespace_ + "/estimator/subscribers/initializerSubscriber/queue_size",
              initializerSubscriberName_);
    // PUBLISHERS
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/publishers/estimator/topic",
              statePublisherName_);
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/publishers/estimator/queue_size",
              statePublisherQueueSize_);

    // PARAMETERS
    // X_O
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/estimatorParameters/X", x_0_);
    x_m_ = x_0_;
    // P
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/estimatorParameters/P", P_init_);
    P_m_ = P_init_ ;
    // Q
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/estimatorParameters/Q", Q_);

    EstimatorBase::readParameters();
  }
  ;

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
    state_ = std_msgs::Float64MultiArray();
    publisherTimer_ = nodeHandle_->createTimer(ros::Duration(0.02),
                                               &KalmanFilterBase::publisherTimerCallback, this);
  }
  ;

  virtual void initialize() override
  {
    x_p_ = Eigen::VectorXd::Zero(n_);
    P_p_ = Eigen::MatrixXd::Zero(n_, n_);
    u_ = Eigen::VectorXd::Zero(m_);
    z_ = Eigen::VectorXd();
    for (auto& s : sensors_) {
      s->initialize();
    }
    EstimatorBase::initialize();
  }
  ;

  virtual void initializeSubscribers() override
  {
    for (auto& s : sensors_) {
      s->initializeSubscribers();
    }
    inputSubcriber_ = nodeHandle_->subscribe(this->namespace_ + "/" + inputSubscriberName_, 10,
                                             &KalmanFilterBase::inputSubscriberCallback, this);

    initializerSubcriber_ = nodeHandle_->subscribe(this->namespace_ + "/" + initializerSubscriberName_,
                                                  initializerSubscriberQueueSize_,
                                                  &KalmanFilterBase::initializerSubscriberCallback,
                                                  this);

    EstimatorBase::initializeSubscribers();

  }
  ;

  virtual void initializePublishers() override
  {
    for (auto& s : sensors_) {
      s->initializePublishers();
    }
    statePublisher_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>(
        this->namespace_ + "/" + statePublisherName_, statePublisherQueueSize_);
  }
  ;

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
  }
  ;

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
  }
  ;

  virtual void publish()
  {
    state_.data.clear();
    for (int i = 0; i < x_m_.size(); i++) {
      state_.data.push_back(x_m_(i));
    }
    statePublisher_.publish(state_);
  }
  ;

  virtual void inputSubscriberCallback(std_msgs::Float64MultiArray msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < msg.data.size(); i++) {
      u_[i] = msg.data[i];
    }
  }
  ;

  virtual void initializerSubscriberCallback(std_msgs::Float64MultiArray msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    //std::cerr << this->ns_ << " init subscriber!!" << std::endl;
    for (int i = 0; i < msg.data.size(); i++) {
      x_m_[i] = msg.data[i];
    }
    //std::cerr<< x_m_.transpose() << std::endl;
  }
  ;

  void publisherTimerCallback(const ros::TimerEvent& event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishTime_ = true;
  }
  ;

 protected:
  int n_;
  int m_;

  std::vector<sensor::SensorHandlerBase*> sensors_;
  std_msgs::Float64MultiArray state_;

  ros::Publisher statePublisher_;
  std::string statePublisherName_;
  int statePublisherQueueSize_;

  ros::Subscriber inputSubcriber_;
  std::string inputSubscriberName_;

  ros::Subscriber initializerSubcriber_;
  std::string initializerSubscriberName_;
  int initializerSubscriberQueueSize_;

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
