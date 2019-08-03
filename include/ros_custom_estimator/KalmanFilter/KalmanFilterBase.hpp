#pragma once

#include "ros_custom_estimator/EstimatorBase.hpp"
#include "ros_custom_estimator/SensorHandler/SensorHandlerBase.hpp"
#include <ctime>

namespace estimator {
class KalmanFilterBase : public EstimatorBase
{
 public:
  KalmanFilterBase(ros::NodeHandle* nodeHandle,
                   hardware_adapter::HardwareAdapterFrameBase& hardwareAdapterFrame,
                   robot::RobotContainerBase& robot)
      : EstimatorBase(nodeHandle, hardwareAdapterFrame, robot),
        m_(0),
        n_(0),
        initializerSubscriberQueueSize_(1)
  {
  }

  //~KalmanFilterBase();

  virtual void create() override
  {
    EstimatorBase::create();

    x_0_ = Eigen::VectorXd::Zero(n_);
    x_m_ = Eigen::VectorXd::Zero(n_);
    P_m_ = Eigen::MatrixXd::Zero(n_, n_);
    P_init_ = Eigen::MatrixXd::Zero(n_, n_);
    Q_ = Eigen::MatrixXd::Zero(n_, n_);
    u_ = Eigen::VectorXd::Zero(m_);
    z_ = Eigen::VectorXd::Zero(n_);

    sensors_ = std::vector<std::unique_ptr<sensor::SensorHandlerBase>>();
    addSensors();
    state_ = std_msgs::Float64MultiArray();
  }

  virtual void readParameters() override
  {

    for (auto& s : sensors_) {
      s->readParameters();
    }
    // SUBSCRIBERS
    paramRead(this->nodeHandle_,
              this->namespace_ + "/estimator/subscribers/initializerSubscriber/topic",
              initializerSubscriberName_);

    paramRead(this->nodeHandle_,
              this->namespace_ + "/estimator/subscribers/initializerSubscriber/queue_size",
              initializerSubscriberQueueSize_);

    // PARAMETERS
    // X_O
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/estimatorParameters/X", x_0_);
    x_m_ = x_0_;
    // P
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/estimatorParameters/P", P_init_);
    P_m_ = P_init_;
    // Q
    paramRead(this->nodeHandle_, this->namespace_ + "/estimator/estimatorParameters/Q", Q_);

    EstimatorBase::readParameters();
  }

  virtual void initializeSubscribers() override
  {
    for (auto& s : sensors_) {
      s->initializeSubscribers();
    }

    initializerSubcriber_ = nodeHandle_->subscribe(
        this->namespace_ + "/" + initializerSubscriberName_, initializerSubscriberQueueSize_,
        &KalmanFilterBase::initializerSubscriberCallback, this);

    EstimatorBase::initializeSubscribers();

  }

  virtual void initializePublishers() override
  {
    for (auto& s : sensors_) {
      s->initializePublishers();
    }
  }

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

  virtual void advance(double dt) override
  {
    for (auto& s : sensors_) {
      s->advance();
    }

    updateInput();

    // Sets the self.z_
    updateMeasurements();

    // Prediction Step
    pStep();
    // Measurement Step
    mStep();
    // Publish the state
    if(active_){
      updateState();
    }
    // Log publish
    publish();
    // Set Measurement and Input flags False
    for (auto& s : sensors_) {
      s->reset();
    }
    
  }

  virtual void updateInput()
  {
    std::lock_guard<std::mutex> lock(
        *this->hardwareAdapterFrame_.getActuator().getActuatorMutex());
    u_ = this->hardwareAdapterFrame_.getActuator().getCommands();
  }

  virtual void updateMeasurements()
  {
    z_ = Eigen::VectorXd();
    Eigen::VectorXd z = Eigen::VectorXd();
    int size_z;
    for (auto& sensor : sensors_) {
      if (sensor->isUpdated()) {
        z = sensor->getMeasurement(x_m_, u_);
        size_z = z_.size();
        z_.conservativeResize(size_z + z.size());
        z_.segment(size_z, z.size()) = z;

        //CONFIRM("Kalm : " + std::to_string(sensor->getTimeStamp()/1000000000)+" , "
        //                  + std::to_string(ros::Time::now().toSec()));
      }
    }
  }

  virtual void pStep()
  {
    //std::cout << "________________" << std::endl;
    //std::cout << "x_m : " << x_m_.transpose() << std::endl;
    //std::cout << "u   : " << u_.transpose() << std::endl;
    calculateF();
    x_p_ = statePrediction(x_m_, u_);
    P_p_ = F_ * P_m_ * F_.transpose() + Q_;
    //std::cout << "x_p : " << x_p_.transpose() << std::endl;
    //std::cout << "P_p : " << P_p_ << std::endl;
  }

  virtual void mStep()
  {
    bool isMeasured = false;
    for (auto& sensor : sensors_) {
      isMeasured |= sensor->isUpdated();
    }
    if (isMeasured) {
      calculateH();
      calculateY();
      Eigen::MatrixXd S = H_ * P_p_ * H_.transpose() + R_;

      K_ = P_p_ * H_.transpose() * S.inverse();
      x_m_ = x_p_ + K_ * y_;
      P_m_ = (Eigen::MatrixXd::Identity(n_, n_) - K_ * H_) * P_p_;
    } else {
      x_m_ = x_p_;
      P_m_ = P_p_;
    }

    //std::cout << "x_m : " << x_m_.transpose() << std::endl;
    //std::cout << "P_m : " << P_m_ << std::endl;
  }

  virtual Eigen::VectorXd statePrediction(Eigen::VectorXd x, Eigen::VectorXd u)
  {
    return F_ * x + B_ * u;
  }

  virtual void calculateF()
  {
    F_ = Eigen::MatrixXd::Identity(n_, n_);
  }

  virtual void calculateH()
  {
    R_ = Eigen::MatrixXd();
    H_ = Eigen::MatrixXd();
    Eigen::MatrixXd H_sensor;
    Eigen::MatrixXd R_sensor;
    int H_rowNum;
    int R_rowNum;
    int R_colNum;
    for (auto& sensor : sensors_) {
      H_sensor = sensor->getObservationJacobian();
      R_sensor = sensor->getObservationNoiseCovariance();
      H_rowNum = H_.rows();
      R_rowNum = R_.rows();
      R_colNum = R_.cols();
      if (sensor->isUpdated()) {
        if (H_.size() == 0) {
          H_ = H_sensor;
          R_ = R_sensor;
        } else {
          H_.conservativeResize(H_rowNum + R_sensor.rows(), H_.cols());
          H_.block(H_rowNum, 0, H_sensor.rows(), H_sensor.cols()) = sensor->getObservationJacobian();
          R_.conservativeResize(R_rowNum + R_sensor.rows(), R_colNum + R_sensor.cols());
          R_.block(R_rowNum, R_colNum, R_sensor.rows(), R_sensor.cols()) = sensor
              ->getObservationNoiseCovariance();
          R_.block(R_rowNum, 0, R_sensor.rows(), R_sensor.cols()) = Eigen::MatrixXd::Zero(
              R_sensor.rows(), R_sensor.cols());
          R_.block(0, R_colNum, R_sensor.rows(), R_sensor.cols()) = Eigen::MatrixXd::Zero(
              R_sensor.rows(), R_sensor.cols());
        }
      }
    }

  }

  virtual void calculateY()
  {

    Eigen::VectorXd x = Eigen::VectorXd();
    Eigen::VectorXd dummy = Eigen::VectorXd();
    int size_x;
    for (auto& sensor : sensors_) {
      if (sensor->isUpdated()) {
        dummy = sensor->getObservation(x_p_);
        size_x = x.size();
        x.conservativeResize(size_x + dummy.size());
        x.segment(size_x, dummy.size()) = dummy;
      }
    }
    y_ = z_ - x;
  }

  virtual void addSensors()
  {
  }

  virtual void updateState()
  {

  }

  virtual void initializerSubscriberCallback(std_msgs::Float64MultiArray msg)
  {
    //std::cerr << this->ns_ << " init subscriber!!" << std::endl;
    for (int i = 0; i < msg.data.size(); i++) {
      x_m_[i] = msg.data[i];
    }
    //std::cerr<< x_m_.transpose() << std::endl;
  }

  virtual void publish()
  {

  }

 protected:
  int n_;
  int m_;

  std::vector<std::unique_ptr<sensor::SensorHandlerBase>> sensors_;
  std_msgs::Float64MultiArray state_;

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

  Eigen::VectorXd y_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd K_;
};
}
