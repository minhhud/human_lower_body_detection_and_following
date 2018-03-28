#ifndef __TRACKER__
#define __TRACKER__

#include "state_pos_vel.h"
#include <leg_msgs/PositionMeasurement.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <string>
#include <leg_detector/LegDetectorConfig.h>
#include <dynamic_reconfigure/server.h>

namespace estimation
{

class Tracker
{
public:
  /// constructor
  Tracker(const std::string& name): name_(name) {};

  /// destructor
  virtual ~Tracker() {};

  /// return the name of the tracker
  const std::string& getName() const {return name_;};

  /// initialize tracker
  virtual void initialize(const BFL::StatePosVel& mu, const BFL::StatePosVel& sigma, const double time) = 0;

  /// return if tracker was initialized
  virtual bool isInitialized() const = 0;

  /// return measure for tracker quality: 0=bad 1=good
  virtual double getQuality() const = 0;

  /// return the lifetime of the tracker
  virtual double getLifetime() const = 0;

  /// return the time of the tracker
  virtual double getTime() const = 0;

  /// update tracker
  virtual bool updatePrediction(const double time) = 0;
  virtual bool updateCorrection(const tf::Vector3& meas,
				const MatrixWrapper::SymmetricMatrix& cov) = 0;

  /// get filter posterior
  virtual void getEstimate(BFL::StatePosVel& est) const = 0;
  virtual void getEstimate(leg_msgs::PositionMeasurement& est) const = 0;

private:
  std::string name_;

}; // class

}; // namespace

#endif

#ifndef __TRACKER_KALMAN__
#define __TRACKER_KALMAN__

// bayesian filtering
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/linearanalyticconditionalgaussian.h>


#include "state_pos_vel.h"

// TF
#include <tf/tf.h>

// log files
#include <fstream>

namespace estimation
{

class TrackerKalman: public Tracker
{
public:
  /// constructor
  TrackerKalman(const std::string& name, const BFL::StatePosVel& sysnoise);

  /// destructor
  virtual ~TrackerKalman();

  virtual void Get_Configure(leg_detector::LegDetectorConfig &config, uint32_t level);
  /// initialize tracker
  virtual void initialize(const BFL::StatePosVel& mu, const BFL::StatePosVel& sigma, const double time);

  /// return if tracker was initialized
  virtual bool isInitialized() const {return tracker_initialized_;};

  /// return measure for tracker quality: 0=bad 1=good
  virtual double getQuality() const {return quality_;};

  /// return the lifetime of the tracker
  virtual double getLifetime() const;

  /// return the time of the tracker
  virtual double getTime() const;

  /// update tracker
  virtual bool updatePrediction(const double time);
  virtual bool updateCorrection(const tf::Vector3& meas,
				const MatrixWrapper::SymmetricMatrix& cov);

  /// get filter posterior
  virtual void getEstimate(BFL::StatePosVel& est) const;
  virtual void getEstimate(leg_msgs::PositionMeasurement& est) const;

  double no_observation_timeout_s_;
  double kal_p_, kal_q_, kal_r_;

private:
  // pdf / model / filter
  BFL::Gaussian                                           prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  BFL::LinearAnalyticConditionalGaussian*                 sys_pdf_;
  BFL::LinearAnalyticSystemModelGaussianUncertainty*      sys_model_;
  BFL::LinearAnalyticConditionalGaussian*                 meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model_;
  MatrixWrapper::Matrix                                   sys_matrix_;
  MatrixWrapper::SymmetricMatrix                          sys_sigma_;

  double calculateQuality();

  // vars
  bool tracker_initialized_;
  double init_time_, filter_time_, quality_;


}; // class

}; // namespace

#endif
