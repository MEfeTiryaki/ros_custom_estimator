#pragma once

#include "ros_custom_estimator/KalmanFilter/KalmanFilterBase.hpp"

namespace estimator {
class ExtendedKalmanFilterBase : public KalmanFilterBase
{
 public:
  ExtendedKalmanFilterBase():
    KalmanFilterBase()
  {
  }
  ;

  ~ExtendedKalmanFilterBase()
  {
  }
  ;

  virtual void pStep() override
  {
    x_p_ = statePrediction(x_m_,u_);
    calculateF();
    P_p_ = F_*P_m_*F_.transpose()+Q_;
  };

  virtual void mStep() override
  {
    bool isMeasured = false;
    for (auto& sensor : sensors_) {
      isMeasured |= sensor->isUpdated();
    }
    if (isMeasured) {
      calculateH();
      calculateY();
      Eigen::MatrixXd S = H_*P_p_*H_.transpose() + R_;

      K_ = P_p_*H_.transpose()*S.inverse();
      x_m_ = x_p_ + K_*y_;
      P_m_ = (Eigen::MatrixXd::Identity(n_,n_)-K_*H_)*P_p_;
      }
    else{
      x_m_ =  x_p_;
      P_m_ =  P_p_;
    }
  }
  ;

  virtual Eigen::VectorXd statePrediction(Eigen::VectorXd x, Eigen::VectorXd u)
  {
    return Eigen::VectorXd::Zero(n_);
  }
  ;
  virtual void calculateF()
  {
    F_ = Eigen::MatrixXd::Identity(n_, n_);
  }
  ;
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
      H_sensor = sensor->getH();
      R_sensor = sensor->getR();
      H_rowNum = H_.rows();
      R_rowNum = R_.rows();
      R_colNum = R_.cols();
      if (sensor->isUpdated()) {
        if (H_.size() == 0) {
          H_ = H_sensor;
          R_ = R_sensor;
        } else {
          H_.conservativeResize(H_rowNum + R_sensor.rows(), H_.cols());
          H_.block(H_rowNum, 0, H_sensor.rows(), H_sensor.cols()) = sensor->getH();
          R_.conservativeResize(R_rowNum + R_sensor.rows(), R_colNum + R_sensor.cols());
          R_.block(R_rowNum, R_colNum, R_sensor.rows(), R_sensor.cols()) = sensor->getR();
          R_.block(R_rowNum, 0, R_sensor.rows(), R_sensor.cols()) = Eigen::MatrixXd::Zero(R_sensor.rows(), R_sensor.cols());
          R_.block(0, R_colNum, R_sensor.rows(), R_sensor.cols()) = Eigen::MatrixXd::Zero(R_sensor.rows(), R_sensor.cols());
        }
      }
    }

  }
  ;
  virtual void calculateY()
  {

    Eigen::VectorXd x = Eigen::VectorXd();
    Eigen::VectorXd dummy = Eigen::VectorXd();
    int size_x ;
    for (auto& sensor : sensors_) {
      if (sensor->isUpdated()) {
        dummy = sensor->geth(x_p_);
        size_x = x.size();
        x.conservativeResize( size_x+dummy.size()) ;
        x.segment(size_x,dummy.size())=dummy;
      }
    }
    y_ = z_ - x;
  }
  ;

 protected:
  Eigen::VectorXd y_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd K_;
};
}
