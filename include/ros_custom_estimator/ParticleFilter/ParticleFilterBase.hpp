/*
 * ParticleFilterBase.hpp
 *
 *  Created on: Jul 7, 2018
 *      Author: efe
 */

#pragma once

#include "ros_custom_estimator/EstimatorBase.hpp"
#include "ros_custom_estimator/SensorHandler/SensorHandlerBase.hpp"
#include "ros_custom_controller_base/State.h"
#include "ros_custom_controller_base/Input.h"
#include <ctime>

namespace estimator {

struct Particle
{
  Eigen::VectorXd xm;
  Eigen::VectorXd xp;
  Eigen::VectorXd wn;
  double bn;
  double Bun;
  double Bln;
};

class ParticleFilterBase : public EstimatorBase
{
 public:
  ParticleFilterBase():
    EstimatorBase(),
    statePublisherQueueSize_(0),
    publishTime_(true),
    defaultProbabilityDensity_(0.0),
    particleNumber_(100),
    n_(0),
    m_(0)
  {
  time_start_ = clock();
  time_stop_ = clock();
  }
  ;

  ~ParticleFilterBase()
  {
  };

  virtual void readParameters() override
  {
    for (auto s : sensors_) {
      s->readParameters();
      if (nodeHandle_->hasParam(
          ns_ + "/estimator/subscribers/sensorSubscriber_" + std::to_string(s->getId()) + "/topic")) {
        std::string name;
        nodeHandle_->getParam(
            ns_ + "/estimator/subscribers/sensorSubscriber_" + std::to_string(s->getId()) + "/topic",
            name);
        std::cout << name << std::endl;
        s->setSubscriberName(name);
      }
    }
    // SUBSCRIBERS
    if (nodeHandle_->hasParam(ns_ + "/estimator/subscribers/inputSubscriber/topic")) {
      nodeHandle_->getParam(ns_ + "/estimator/subscribers/inputSubscriber/topic",
                            inputSubscriberName_);
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
    // Particle Number_
    if (nodeHandle_->hasParam(ns_ + "/estimator/PF/particle_number")) {
      nodeHandle_->getParam(ns_ + "/estimator/PF/particle_number", particleNumber_);
    }

    EstimatorBase::readParameters();
  };

  virtual void create() override
  {
    sensors_ = std::vector<sensor::SensorHandlerBase*>();
    addSensors();
    for (auto& s : sensors_) {
      s->create();
    }
    state_ = ros_custom_controller_base::State();
    publisherTimer_ = nodeHandle_->createTimer(ros::Duration(0.02),
                                               &ParticleFilterBase::publisherTimerCallback, this);

    randomNumberGenerator_ = std::default_random_engine(randomDevice_());
    uniformDistribution_ = std::uniform_real_distribution<double>(0.0, 1.0);

  };

  virtual void initilize() override
  {

    for (auto& s : sensors_) {
       s->initilize(nodeHandle_);
     }
    EstimatorBase::initilize();
    particles_ = std::vector<Particle>(particleNumber_);
    resamplingParticles_ = std::vector<Particle>(particleNumber_);
    double sum = 0.0;
    Eigen::VectorXd randomVector = Eigen::VectorXd::Zero(n_);
    for (auto& p : particles_) {
      for (int i = 0; i < n_; i++) {
        randomVector[i] = uniformDistribution_(randomNumberGenerator_);
      }
      p.xm = initilizationDistribution(randomVector);
      p.bn = defaultProbabilityDensity_;
      p.Bln = sum;
      sum += p.bn;
      p.Bun = sum;
    }
    sum = 0.0;
    for (auto& p : resamplingParticles_) {
      p.xm = Eigen::VectorXd::Zero(n_);
      p.bn = defaultProbabilityDensity_;
      p.Bln = sum;
      sum += p.bn;
      p.Bun = sum;
    }
    x_ = Eigen::VectorXd::Zero(n_);
    u_ = Eigen::VectorXd::Zero(m_);
    defaultProbabilityDensity_ = 1.0 / particleNumber_;

  };

  virtual void initilizeSubscribers() override
  {
    for (auto& s : sensors_) {
      s->initilizeSubscribers();
    }
    inputSubcriber_ = nodeHandle_->subscribe(ns_ + "/" + inputSubscriberName_, 10,
                                             &ParticleFilterBase::inputSubscriberCallback, this);
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
    //
    getMeasurements(dt_);
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
    std::cout << "Time: " << (time_stop_ - time_start_) / double(CLOCKS_PER_SEC) << " sec "
              << std::endl;
    time_start_ = clock();
  }
;

  virtual void addSensors()
  {
  };

  virtual void pStep()
  {
    Eigen::VectorXd randomVector = Eigen::VectorXd::Zero(n_);
    for (auto& p : particles_) {
      for (int i = 0; i < n_; i++) {
        randomVector[i] = uniformDistribution_(randomNumberGenerator_);
      }
      p.xp = predict(p.xm, u_, processNoise(randomVector));
      //std::cerr<< "p.xp : "<<p.xp.transpose() << std::endl;
      //std::cerr<< "p.xm : "<<p.xm.transpose() << std::endl;
    }
  };

  virtual void mStep()
  {
    double sum = 0.0;
    double randomNumber = 0.0;
    int searchIndex = -1;
    int newSearchIndex = -1;
    bool isMeasured = false;
    for (auto& sensor : sensors_) {
      isMeasured |= sensor->isUpdated();
    }
    if (isMeasured) {
      for (auto& p : particles_) {
        // calculate probablity dense of given noise
        p.bn = defaultProbabilityDensity_;
        for (auto& s : sensors_) {
          if (s->isUpdated()) {
            p.bn *= s->probabilityDensity(p.xp);
          }
        }
        p.Bln = sum;
        // update sume
        sum += p.bn;
        // add PD to sum
        p.Bun = sum;
      }

      Eigen::VectorXd expectedValue = Eigen::VectorXd::Zero(n_);
      for (auto& p : particles_) {
        // Normalize
        p.bn /= sum;
        p.Bun /= sum;
        p.Bln /= sum;
        // expected value
        expectedValue += p.bn * p.xp ;
        //std::cerr << "p.b :" << p.bn << " , " << p.Bun << " , " << p.Bln << std::endl;
      }
      x_ = expectedValue;
    }
    // Resampling
    sum = 0.0;
    for (auto& p : resamplingParticles_) {
      randomNumber = uniformDistribution_(randomNumberGenerator_);
      searchIndex = randomNumber * particleNumber_ + 0.5;

      int kill = 0;
      while (true) {
        if (kill++ > 50) {
          break;
        }

        if (particles_[searchIndex - 1].Bun > randomNumber) {
          if (randomNumber > particles_[searchIndex - 1].Bln) {
            // In the desired range DONE
            break;
          } else {
            // Below of the desired value XXX: 0.5 for rounding closest value
            newSearchIndex = searchIndex
                - (int) ((particles_[searchIndex - 1].Bun - randomNumber) / searchIndex + 0.5);

            if (searchIndex == newSearchIndex) {
              searchIndex = newSearchIndex - 1;
            } else {
              searchIndex = newSearchIndex;
            }
          }
        } else {
          //  Above the desired value
          newSearchIndex = searchIndex
              + (int) ((randomNumber - particles_[searchIndex - 1].Bun)
                  / (particleNumber_ - searchIndex) + 0.5);

          if (searchIndex == newSearchIndex) {
            searchIndex = newSearchIndex + 1;
          } else {
            searchIndex = newSearchIndex;
          }
        }
      }
      p.xm = particles_[searchIndex - 1].xp;
      p.bn = defaultProbabilityDensity_;
      p.Bln = sum;
      sum += p.bn;
      p.Bun = sum;
    }
    // Roughtening

    // Update particles
    particles_ = resamplingParticles_;
  };

  virtual void getMeasurements(double dt)
  {
    for (auto& sensor : sensors_) {
      if (sensor->isUpdated()) {
        sensor->getData(x_, u_);
      }
    }
  };

  virtual Eigen::VectorXd predict(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd v)
  {
    return Eigen::VectorXd();
  }
  ;
  virtual Eigen::VectorXd initilizationDistribution(Eigen::VectorXd v)
  {
    return Eigen::VectorXd();
  }
  ;

  virtual Eigen::VectorXd processNoise(Eigen::VectorXd v)
  {
    return Eigen::VectorXd();
  }
  ;


  virtual void publish()
  {
    state_.header.stamp = ros::Time::now();
    state_.state.clear();
    for (int i = 0; i < n_; i++) {
      state_.state.push_back(x_[i]);
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
  ros::Publisher vis_pub;

  ros::Subscriber inputSubcriber_;
  std::string inputSubscriberName_;

  int particleNumber_;
  std::vector<Particle> particles_;
  std::vector<Particle> resamplingParticles_;
  double defaultProbabilityDensity_;
  Eigen::VectorXd x_;

  std::random_device randomDevice_;
  std::default_random_engine randomNumberGenerator_;
  std::uniform_real_distribution<double> uniformDistribution_;
  Eigen::VectorXd u_;

  Eigen::VectorXd initialDistirubtionCoverriance_ ;
  Eigen::VectorXd processNoiseCoveriance_;
  ros::Timer publisherTimer_;
  bool publishTime_;

  double time_start_;
  double time_stop_;
};
}
