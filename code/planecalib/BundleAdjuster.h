/*
 * BundleAdjuster.h
 *
 *  Created on: 6.3.2014
 *      Author: dan
 */

#ifndef BUNDLEADJUSTER_H_
#define BUNDLEADJUSTER_H_

#include <vector>
#include <unordered_set>
#include <memory>
#include <opencv2/core.hpp>
#include "Map.h"

namespace planecalib {

class PoseTracker;

class BundleAdjuster
{
public:
	BundleAdjuster():
		mUseLocks(true), mIsExpanderBA(false), mTracker(NULL)
	{}

	void setOutlierThreshold(float pixelThreshold)
	{
		mOutlierPixelThreshold = pixelThreshold;
		mOutlierPixelThresholdSq = pixelThreshold*pixelThreshold;
	}

	bool getUseLocks() const {return mUseLocks;}
	void setUseLocks(bool value) {mUseLocks = value;}

	void setIsExpanderBA(bool value) { mIsExpanderBA = value; }

	const std::unordered_set<Keyframe *> getFramesToAdjust() const {return mFramesToAdjust;}
	const std::unordered_set<Feature *> getFeaturesToAdjust() const {return mFeaturesToAdjust;}

	void setMap(Map *map) {mMap=map;}
	void setTracker(PoseTracker *tracker) { mTracker = tracker; }

	void addFrameToAdjust(Keyframe &newFrame);

	bool isInlier(const FeatureMeasurement &measurement, const Eigen::Matrix3dr &pose, const Eigen::Vector2d &position);

	void getInliers(const std::unordered_map<Keyframe *, Eigen::Matrix3dr> &paramsPoses,
		const std::unordered_map<Feature *, Eigen::Vector2d> &paramsFeatures,
			const std::vector<FeatureMeasurement> &measurements,
			int &inlierCount);

	bool bundleAdjust();

protected:
	float mOutlierPixelThreshold;
	float mOutlierPixelThresholdSq;
	bool mUseLocks;
	bool mIsExpanderBA;

	Map *mMap;
	PoseTracker *mTracker; //resync() will be called on this tracker if the BA is succesful
	std::unordered_set<Keyframe *> mFramesToAdjust;
	std::unordered_set<Feature *> mFeaturesToAdjust;

	typedef std::pair<std::unordered_map<Keyframe *, Eigen::Matrix3dr>::iterator, bool> TGetPoseParamsResult;
	TGetPoseParamsResult getPoseParams(Keyframe &frame, std::unordered_map<Keyframe *, Eigen::Matrix3dr> &paramsPoses);
};

}

#endif /* BUNDLEADJUSTER_H_ */
