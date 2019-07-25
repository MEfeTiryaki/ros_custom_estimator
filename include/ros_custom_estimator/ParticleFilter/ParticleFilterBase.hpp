/*
 * ParticleFilterBase.hpp
 *
 *  Created on: Jul 7, 2018
 *      Author: efe
 */

#pragma once

#include "ros_custom_estimator/EstimatorBase.hpp"
#include "ros_custom_estimator/SensorHandler/ParticleFilterSensorHandlerBase.hpp"

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
  ParticleFilterBase(ros::NodeHandle* nodeHandle,
                   hardware_adapter::HardwareAdapterFrameBase& hardwareAdapterFrame,
                   robot::RobotContainerBase& robot):
    EstimatorBase(nodeHandle,hardwareAdapterFrame,robot),
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
    for (auto& s : sensors_) {
      s->readParameters();
    }
    // Particle Number_
    if (nodeHandle_->hasParam(this->namespace_ + "/estimator/PF/particle_number")) {
      nodeHandle_->getParam(this->namespace_ + "/estimator/PF/particle_number", particleNumber_);
    }

    EstimatorBase::readParameters();
  };

  virtual void create() override
  {
    sensors_ = std::vector<std::unique_ptr<sensor::ParticleFilterSensorHandlerBase>>();
    addSensors();
    for (auto& s : sensors_) {
      s->create();
    }


    randomNumberGenerator_ = std::default_random_engine(randomDevice_());
    uniformDistribution_ = std::uniform_real_distribution<double>(0.0, 1.0);

  };

  virtual void initialize() override
  {

    for (auto& s : sensors_) {
       s->initialize();
     }
    EstimatorBase::initialize();
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

  virtual void initializeSubscribers() override
  {
    for (auto& s : sensors_) {
      s->initializeSubscribers();
    }
    EstimatorBase::initializeSubscribers();

  }

  virtual void initializePublishers() override
  {
    for (auto& s : sensors_) {
      s->initializePublishers();
    }

  }

  virtual void advance(double dt) override
  {
    //std::lock_guard<std::mutex> lock(mutex_);

    updateInput();
    // Set Measurement and Input flags False
    for (auto& s : sensors_) {
      s->reset();
    }
    //
    getMeasurements(dt_);
    // Prediction Step
    pStep();
    // Measurement Step
    mStep();
    // Publish the state
    publish();

    //time_stop_ = clock();
    //std::cout << "Time: " << (time_stop_ - time_start_) / double(CLOCKS_PER_SEC) << " sec "
    //          << std::endl;
    //time_start_ = clock();
  }


  virtual void addSensors()
  {
  }

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
  }

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

  virtual void updateInput()
  {

  }

  virtual void getMeasurements(double dt)
  {

  }

  virtual Eigen::VectorXd predict(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd v)
  {
    return Eigen::VectorXd();
  }

  virtual Eigen::VectorXd initilizationDistribution(Eigen::VectorXd v)
  {
    return Eigen::VectorXd();
  }

  virtual Eigen::VectorXd processNoise(Eigen::VectorXd v)
  {
    return Eigen::VectorXd();
  }

  virtual void updateState()
  {
    x_ = Eigen::VectorXd::Zero(n_);
    for(auto p : particles_){
      x_ += p.xm;
    }
    x_ /= particles_.size();
  }

  virtual void publish()
  {

  }


 protected:
  int n_;
  int m_;

  std::vector<std::unique_ptr<sensor::ParticleFilterSensorHandlerBase>> sensors_;

  int statePublisherQueueSize_;
  ros::Publisher vis_pub;

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
