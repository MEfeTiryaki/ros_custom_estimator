#pragma once

#include "ros_custom_estimator/EstimatorBase.hpp"

#include <Eigen/SparseCore>
#include <ctime>

namespace estimator {
class KalmanFilterBase : public EstimatorBase {
public:
  KalmanFilterBase(ros::NodeHandle *nodeHandle,
                    SensorBase &sensor, ActuatorBase &actuator,
                   robot::RobotContainerBase &robot)
      : EstimatorBase(nodeHandle,   sensor, actuator, robot), m_(0), n_(0),
        initializerSubscriberQueueSize_(1), isMeasurementUpdated_(false) {}

  //~KalmanFilterBase();

  virtual void create() override {
    EstimatorBase::create();

    x_0_ = Eigen::VectorXd::Zero(n_);
    x_m_ = Eigen::VectorXd::Zero(n_);
    P_m_ = Eigen::SparseMatrix<double, Eigen::RowMajor>();
    P_m_.resize(n_, n_);
    P_m_.setZero();
    P_init_ = Eigen::SparseMatrix<double, Eigen::RowMajor>();
    P_init_.resize(n_, n_);
    P_init_.setZero();
    Q_ = Eigen::SparseMatrix<double, Eigen::RowMajor>();
    Q_.resize(n_, n_);
    Q_.setZero();
    F_ = Eigen::SparseMatrix<double, Eigen::RowMajor>();
    F_.resize(n_, n_);
    F_.setZero();
    B_ = Eigen::SparseMatrix<double, Eigen::RowMajor>();
    B_.resize(n_, m_);
    B_.setZero();
    IdentityNxN_ = Eigen::SparseMatrix<double, Eigen::RowMajor>();
    IdentityNxN_.resize(n_, n_);
    IdentityNxN_.setIdentity();

    u_ = Eigen::VectorXd::Zero(m_);
    z_ = Eigen::VectorXd::Zero(n_);

    sensors_ = std::vector<std::unique_ptr<sensor::SensorHandlerBase>>();
    addSensors();
    state_ = std_msgs::Float64MultiArray();
  }

  virtual void readParameters() override {

    for (auto &s : sensors_) {
      s->readParameters();
    }
    // SUBSCRIBERS
    paramRead(this->nodeHandle_,
              "/" + this->namespace_ +
                  "/estimator/subscribers/initializerSubscriber/topic",
              initializerSubscriberName_);

    paramRead(this->nodeHandle_,
              "/" + this->namespace_ +
                  "/estimator/subscribers/initializerSubscriber/queue_size",
              initializerSubscriberQueueSize_);

    // PARAMETERS
    // X_O
    paramRead(this->nodeHandle_,
              "/" + this->namespace_ + "/estimator/estimatorParameters/X",
              x_0_);
    x_m_ = x_0_;
    // P
    Eigen::MatrixXd P;
    paramRead(this->nodeHandle_,
              "/" + this->namespace_ + "/estimator/estimatorParameters/P", P);
    this->P_init_ = P.sparseView();
    P_m_ = P_init_;
    // Q
    Eigen::MatrixXd Q;
    paramRead(this->nodeHandle_,
              "/" + this->namespace_ + "/estimator/estimatorParameters/Q", Q);
    this->Q_ = Q.sparseView();
    EstimatorBase::readParameters();
  }

  virtual void initializeSubscribers() override {
    for (auto &s : sensors_) {
      s->initializeSubscribers();
    }

    initializerSubcriber_ = nodeHandle_->subscribe(
        "/" + this->namespace_ + "/" + initializerSubscriberName_,
        initializerSubscriberQueueSize_,
        &KalmanFilterBase::initializerSubscriberCallback, this);

    EstimatorBase::initializeSubscribers();
  }

  virtual void initializePublishers() override {
    for (auto &s : sensors_) {
      s->initializePublishers();
    }
  }

  virtual void initialize() override {
    x_p_ = Eigen::VectorXd::Zero(n_);
    P_p_.resize(n_, n_);
    P_p_.setZero();

    u_ = Eigen::VectorXd::Zero(m_);
    z_ = Eigen::VectorXd();

    for (auto &s : sensors_) {
      s->initialize();
    }
    EstimatorBase::initialize();
  }

  virtual void advance(double dt) override {
    for (auto &s : sensors_) {
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
    if (active_) {
      updateState();
    }
    // Log publish
    publish();
    // Set Measurement and Input flags False
    for (auto &s : sensors_) {
      s->reset();
    }
  }

  virtual void updateInput() {
    std::lock_guard<std::mutex> lock(
        *actuator_.getActuatorMutex());
    u_ = actuator_.getCommand();
  }

  virtual void updateMeasurements() {
    z_ = Eigen::VectorXd();
    Eigen::VectorXd z = Eigen::VectorXd();
    int size_z;
    hRows_ = 0;
    rRows_ = 0;
    for (auto &sensor : sensors_) {
      if (sensor->isUpdated()) {
        z = sensor->getMeasurement(x_m_, u_);
        size_z = z_.size();
        z_.conservativeResize(size_z + z.size());
        z_.segment(size_z, z.size()) = z;
        hRows_ += sensor->getObservationJacobianRows();
        rRows_ += sensor->getObservationNoiseCovarianceRows();
        // CONFIRM("Kalm : " +
        // std::to_string(sensor->getTimeStamp()/1000000000)+" , "
        //                  + std::to_string(ros::Time::now().toSec()));
        this->isMeasurementUpdated_ = true;
      }
    }
    checkMeasurement();
  }

  virtual void pStep() {
    calculateF();
    calculateB();

    x_p_ = statePrediction(x_m_, u_);
    P_p_ = (Eigen::SparseMatrix<double, Eigen::RowMajor>)(F_ * (P_m_ *
                                                                F_.transpose())
                                                                   .pruned())
               .pruned() +
           Q_;
  }

  virtual void mStep() {
    if (isMeasurementUpdated_) {
      calculateH();
      calculateY();
      Eigen::MatrixXd S = Eigen::MatrixXd(
          (Eigen::SparseMatrix<double, Eigen::RowMajor>)((H_ * P_p_ *
                                                          H_.transpose())
                                                             .pruned())
              .pruned() +
          R_);
      K_ = (Eigen::SparseMatrix<double, Eigen::RowMajor>)(P_p_ * H_.transpose())
               .pruned() *
           S.inverse().sparseView();
      x_m_ = x_p_ + K_ * y_;
      P_m_ = (Eigen::SparseMatrix<
              double, Eigen::RowMajor>)((IdentityNxN_ -
                                         (Eigen::SparseMatrix<
                                             double, Eigen::RowMajor>)(K_ * H_)
                                             .pruned()) *
                                        P_p_)
                 .pruned();
    } else {
      x_m_ = x_p_;
      P_m_ = P_p_;
    }
  }

  bool checkMeasurement() {
    isMeasurementUpdated_ = false;
    for (auto &sensor : sensors_) {
      isMeasurementUpdated_ |= sensor->isUpdated();
    }
    return isMeasurementUpdated_;
  }

  virtual Eigen::VectorXd statePrediction(Eigen::VectorXd x,
                                          Eigen::VectorXd u) {
    return F_ * x + B_ * u;
  }

  virtual void calculateF() { F_ = IdentityNxN_; }

  virtual void calculateB() {}

  virtual void calculateH() {
    // TODO : R can be diagonal
    R_.resize(rRows_, rRows_);
    H_.resize(hRows_, n_);
    R_.setZero();
    H_.setZero();
    Eigen::MatrixXd H_sensor;
    Eigen::MatrixXd R_sensor;
    //  int H_rowNum;
    // int R_rowNum;
    // int R_colNum;
    int hRowIndex = 0;
    int rRowIndex = 0;
    Eigen::MatrixXd rRowMatrix = Eigen::MatrixXd::Zero(1, rRows_);
    for (auto &sensor : sensors_) {

      if (sensor->isUpdated()) {
        H_sensor = sensor->getObservationJacobian();
        R_sensor = sensor->getObservationNoiseCovariance();
        rRowMatrix = Eigen::MatrixXd::Zero(R_sensor.rows(), rRows_);
        rRowMatrix.block(0, rRowIndex, R_sensor.rows(), R_sensor.rows()) =
            R_sensor;
        // SET H
        H_.middleRows(hRowIndex, H_sensor.rows()) = H_sensor.sparseView();
        hRowIndex += H_sensor.rows();
        // SET R
        R_.middleRows(rRowIndex, R_sensor.rows()) = rRowMatrix.sparseView();
        rRowIndex += R_sensor.rows();
        /*
       if (H_.size() == 0) {
         H_ = H_sensor.sparseView();
         R_ = R_sensor.sparseView();
       } else {
         H_.conservativeResize(H_rowNum + R_sensor.rows(), H_.cols());
         H_.block(H_rowNum, 0, H_sensor.rows(), H_sensor.cols()) =
             sensor->getObservationJacobian();

         R_.conservativeResize(R_rowNum + R_sensor.rows(),
                               R_colNum + R_sensor.cols());
         R_.block(R_rowNum, R_colNum, R_sensor.rows(), R_sensor.cols()) =
             sensor->getObservationNoiseCovariance();
         R_.block(R_rowNum, 0, R_sensor.rows(), R_sensor.cols()) =
             Eigen::MatrixXd::Zero(R_sensor.rows(), R_sensor.cols());
         R_.block(0, R_colNum, R_sensor.rows(), R_sensor.cols()) =
             Eigen::MatrixXd::Zero(R_sensor.rows(), R_sensor.cols());
       }
       */
      }
    }
  }

  virtual void calculateY() {

    Eigen::VectorXd x = Eigen::VectorXd();
    Eigen::VectorXd dummy = Eigen::VectorXd();
    int size_x;
    for (auto &sensor : sensors_) {
      if (sensor->isUpdated()) {
        dummy = sensor->getObservation(x_p_);
        size_x = x.size();
        x.conservativeResize(size_x + dummy.size());
        x.segment(size_x, dummy.size()) = dummy;
      }
    }
    y_ = z_ - x;
  }

  virtual void addSensors() {}

  virtual void updateState() {}

  virtual void initializerSubscriberCallback(std_msgs::Float64MultiArray msg) {
    // std::cerr << this->ns_ << " init subscriber!!" << std::endl;
    for (int i = 0; i < msg.data.size(); i++) {
      x_m_[i] = msg.data[i];
    }
    // std::cerr<< x_m_.transpose() << std::endl;
  }

  virtual void publish() {}

protected:
  int n_;
  int m_;

  bool isMeasurementUpdated_;

  std_msgs::Float64MultiArray state_;

  ros::Subscriber initializerSubcriber_;
  std::string initializerSubscriberName_;
  int initializerSubscriberQueueSize_;

  Eigen::VectorXd x_p_;
  Eigen::VectorXd x_m_;
  Eigen::VectorXd x_0_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> P_p_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> P_m_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> P_init_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q_;

  Eigen::VectorXd u_;
  Eigen::VectorXd z_;
  Eigen::VectorXd y_;

  Eigen::SparseMatrix<double, Eigen::RowMajor> F_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> B_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> H_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> R_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> K_;

  int hRows_;
  int rRows_;

  Eigen::SparseMatrix<double, Eigen::RowMajor> IdentityNxN_;
};
} // namespace estimator
