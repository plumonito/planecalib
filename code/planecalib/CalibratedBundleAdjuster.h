
#ifndef CALIBRATEDBUNDLEADJUSTER_H_
#define CALIBRATEDBUNDLEADJUSTER_H_

#include "eutils.h"
#include <vector>
#include <unordered_set>
#include <memory>
#include <opencv2/core.hpp>
#include "Map.h"

namespace planecalib {

class PoseTracker;

class CalibratedBundleAdjuster
{
public:
	CalibratedBundleAdjuster() :
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
	
	void setK(const Eigen::Matrix3dr &K) { mK = K; }
	const Eigen::Matrix3dr &getK() const { return mK; }

	void addFrameToAdjust(Keyframe &newFrame);

	bool bundleAdjust();

protected:
	float mOutlierPixelThreshold;
	float mOutlierPixelThresholdSq;
	bool mUseLocks;

	Map *mMap;
	std::unordered_set<Keyframe *> mFramesToAdjust;
	std::unordered_set<Feature *> mFeaturesToAdjust;

	Eigen::Matrix3dr mK;

	Eigen::Vector3d mParamsK;
	std::unordered_map<Keyframe *, Eigen::Matrix<double, 1, 6>, std::hash<Keyframe*>, std::equal_to<Keyframe*>, Eigen::aligned_allocator<std::pair<Keyframe*,Eigen::Matrix<double,1,6>>>> mParamsPoses;
	std::unordered_map<Feature *, Eigen::Vector2d, std::hash<Feature*>, std::equal_to<Feature*>, Eigen::aligned_allocator<std::pair<Feature*, Eigen::Vector2d>>> mParamsFeatures;
	std::vector<FeatureMeasurement *> mMeasurementsInProblem;

	Eigen::Matrix<double, 1, 6> &getPoseParams(Keyframe *framep);
	Eigen::Vector2d &getFeatureParams(Feature *featurep);

	bool isInlier(const FeatureMeasurement &measurement, const Eigen::Matrix<double,1,6> &pose, const Eigen::Vector2d &position);

	void getInliers(int &inlierCount);
};

}

#endif /* BUNDLEADJUSTER_H_ */
