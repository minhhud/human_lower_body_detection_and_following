#ifndef __TRACKED_FEATURE_
#define __TRACKED_FEATURE_

#include "tracker/tracker_kalman.h"
#include "tracker/state_pos_vel.h"
#include "detector/laser_processor.h"
#include "leg_detector/LegDetectorConfig.h"
#include <dynamic_reconfigure/server.h>

using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

class SingleLegFeature
{
public:
	Stamped<Point> 		loc_;
	cv::Rect			pixel_;
	double 				probability_;
	SingleLegFeature* 	other_;
	std::string 		object_id_;

	SingleLegFeature(Stamped<Point> loc, double probability)
	: loc_(tf::Point(loc.getY(), loc.getZ(), loc.getX()), loc.stamp_, loc.frame_id_), probability_(probability)
	{}

	double getProbability()
	{
		return probability_;
	}
};

class LegsFeature
{
public:
	SingleLegFeature* 	leg1_;
	SingleLegFeature* 	leg2_;
	Stamped<Point> 		center_;
	std::string 		direction_;
	cv::Rect			pixel_;
	int*				hist_;
	int					undetected_time_;
	static int 			nextid;
	BFL::StatePosVel 	sys_sigma_;
	TrackerKalman 		filter_;
	std::string 		object_id;
	double 				reliability, p;
	ros::Time 			time_;
	ros::Time 			meas_time_;
	LegsFeature* 		other;

	LegsFeature(SingleLegFeature* leg1, SingleLegFeature* leg2, cv::Rect pixel, std::string dir, int* hist)
	: leg1_(leg1), leg2_(leg2), direction_(dir), hist_(hist)
	, sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)), pixel_(pixel), undetected_time_(0)
	, filter_("tracker_name",sys_sigma_), reliability(std::min(leg1->getProbability(), leg2->getProbability())), p(4)
	{
//		center_ = std::min(loc.begin(), loc.end()) + (std::max(loc.begin(), loc.end()) - std::min(loc.begin(), loc.end()));
		float cx = std::min(leg1_->loc_.getX(), leg2_->loc_.getX()) + (std::max(leg1_->loc_.getX(), leg2_->loc_.getX()) - std::min(leg1_->loc_.getX(), leg2_->loc_.getX()))/2;
		float cy = std::min(leg1_->loc_.getY(), leg2_->loc_.getY()) + (std::max(leg1_->loc_.getY(), leg2_->loc_.getY()) - std::min(leg1_->loc_.getY(), leg2_->loc_.getY()))/2;
		float cz = std::min(leg1_->loc_.getZ(), leg2_->loc_.getZ()) + (std::max(leg1_->loc_.getZ(), leg2_->loc_.getZ()) - std::min(leg1_->loc_.getZ(), leg2_->loc_.getZ()))/2;

		Stamped<tf::Point> loc(tf::Point(cx, cy, cz), std::max(leg1_->loc_.stamp_, leg2_->loc_.stamp_), leg1_->loc_.frame_id_);

		object_id = "";
		time_ = std::max(leg1_->loc_.stamp_, leg2_->loc_.stamp_);
		meas_time_ = std::max(leg1_->loc_.stamp_, leg2_->loc_.stamp_);

		other = NULL;
		StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
		filter_.initialize(loc, prior_sigma, time_.toSec());

		StatePosVel est;
		filter_.getEstimate(est);

		updatePosition();
	}

	void propagate(ros::Time time)
	{
		time_ = time;

		filter_.updatePrediction(time.toSec());

		updatePosition();
	}

	void update(LegsFeature* leg)
	{
		meas_time_ = leg->center_.stamp_;
		time_ = meas_time_;

		leg1_ = leg->leg1_;
		leg2_ = leg->leg2_;
		direction_ = leg->direction_;

		undetected_time_ = leg->undetected_time_;
		hist_ = leg->hist_;
		pixel_ = leg->pixel_;
		SymmetricMatrix cov(3);
		cov = 0.0;
		cov(1,1) = 0.0025;
		cov(2,2) = 0.0025;
		cov(3,3) = 0.0025;

		filter_.updateCorrection(leg->center_, cov);

		updatePosition();

		if(reliability<0)
		{
			reliability = leg->reliability;
			p = filter_.kal_p_;
		}
		else
		{
			p += filter_.kal_q_;
			double k = p / (p+filter_.kal_r_);
			reliability += k * (leg->reliability - reliability);
			p *= (1 - k);
		}
	}

	double getReliability()
	{
		return reliability;
	}

	double getLifetime()
	{
		return filter_.getLifetime();
	}
private:
	void updatePosition()
	{
		StatePosVel est;
		filter_.getEstimate(est);

		center_[0] = est.pos_[0];
		center_[1] = est.pos_[1];
		center_[2] = est.pos_[2];

		center_.stamp_ = time_;
		double nreliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));
	}
};


class MatchedFeature
{
public:
	LegsFeature* candidate_;
	LegsFeature* closest_;
	float distance_;
	double probability_;

	MatchedFeature(LegsFeature* candidate, LegsFeature* closest, float distance)
	: candidate_(candidate)
	, closest_(closest)
	, distance_(distance)
	{
		probability_ = candidate->getReliability();
	}

	inline bool operator< (const MatchedFeature& b) const
	{
		return (distance_ <  b.distance_);
	}
};
#endif
