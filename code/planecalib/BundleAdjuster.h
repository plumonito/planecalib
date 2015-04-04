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
		mUseLocks(true)
	{}

	void setOutlierThreshold(float pixelThreshold)
	{
		mOutlierPixelThreshold = pixelThreshold;
		mOutlierPixelThresholdSq = pixelThreshold*pixelThreshold;
	}

	bool getUseLocks() const {return mUseLocks;}
	void setUseLocks(bool value) {mUseLocks = value;}

	const std::unordered_set<Keyframe *> getFramesToAdjust() const {return mFramesToAdjust;}
	const std::unordered_set<Feature *> getFeaturesToAdjust() const {return mFeaturesToAdjust;}

	void setMap(Map *map) {mMap=map;}

	void addFrameToAdjust(Keyframe &newFrame);

	bool bundleAdjust();

protected:
	float mOutlierPixelThreshold;
	float mOutlierPixelThresholdSq;
	bool mUseLocks;
	bool mIsExpanderBA;

	Map *mMap;
	std::unordered_set<Keyframe *> mFramesToAdjust;
	std::unordered_set<Feature *> mFeaturesToAdjust;

	std::unordered_map<Keyframe *, Eigen::Matrix3dr, std::hash<Keyframe*>, std::equal_to<Keyframe*>, Eigen::aligned_allocator<std::pair<Keyframe*, Eigen::Matrix3dr>>> mParamsPoses;
	std::unordered_map<Feature *, Eigen::Vector2d, std::hash<Feature*>, std::equal_to<Feature*>, Eigen::aligned_allocator<std::pair<Feature*, Eigen::Vector2d>>> mParamsFeatures;
	std::vector<FeatureMeasurement *> mMeasurementsInProblem;

	Eigen::Matrix3dr &getPoseParams(Keyframe *framep);
	Eigen::Vector2d &getFeatureParams(Feature *featurep);


	bool isInlier(const FeatureMeasurement &measurement, const Eigen::Matrix3dr &pose, const Eigen::Vector2d &position);

	void getInliers(int &inlierCount);
};

}

#endif /* BUNDLEADJUSTER_H_ */
