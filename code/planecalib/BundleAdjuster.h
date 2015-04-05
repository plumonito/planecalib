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
		mUseLocks(true), mOnlyDistortion(false), mParamsDistortion(0,0)
	{}

	void setOutlierThreshold(float pixelThreshold)
	{
		mOutlierPixelThreshold = pixelThreshold;
		mOutlierPixelThresholdSq = pixelThreshold*pixelThreshold;
	}

	bool getUseLocks() const {return mUseLocks;}
	void setUseLocks(bool value) {mUseLocks = value;}

	bool getOnlyDistortion() const { return mOnlyDistortion; }
	void setOnlyDistortion(bool value) { mOnlyDistortion = value; }

	void setMap(Map *map) { mMap = map; }

	const std::unordered_set<Keyframe *> getFramesToAdjust() const {return mFramesToAdjust;}
	const std::unordered_set<Feature *> getFeaturesToAdjust() const {return mFeaturesToAdjust;}

	const Eigen::Vector2d &getDistortion() const { return mParamsDistortion; }
	void setDistortion(const Eigen::Vector2d &coeff) { mParamsDistortion = coeff; }

	const Eigen::Vector2d &getP0() const { return mParamsP0; }
	void setP0(const Eigen::Vector2d &p0) { mParamsP0 = p0; } 

	void addFrameToAdjust(Keyframe &newFrame);

	bool bundleAdjust();

protected:
	float mOutlierPixelThreshold;
	float mOutlierPixelThresholdSq;
	bool mUseLocks;
	bool mOnlyDistortion;

	Map *mMap;
	std::unordered_set<Keyframe *> mFramesToAdjust;
	std::unordered_set<Feature *> mFeaturesToAdjust;

	Eigen::Vector2i mImageSize;

	Eigen::Vector2d mParamsDistortion;
	Eigen::Vector2d mParamsP0;
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
