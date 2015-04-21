
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
		mUseLocks(true), mFixK(false), mFixDistortion(false), mFix3DPoints(false), mParamsDistortion(0, 0), mK(Eigen::Matrix3dr::Identity()), mOutlierPixelThreshold(3), mOutlierPixelThresholdSq(9)
	{}

	void setOutlierThreshold(float pixelThreshold)
	{
		mOutlierPixelThreshold = pixelThreshold;
		mOutlierPixelThresholdSq = pixelThreshold*pixelThreshold;
	}

	bool getUseLocks() const {return mUseLocks;}
	void setUseLocks(bool value) {mUseLocks = value;}

	bool getFixK() const { return mFixK; }
	void setFixK(bool value) { mFixK = value; }

	bool getFixDistortion() const { return mFixDistortion; }
	void setFixDistortion(bool value) { mFixDistortion = value; }

	bool getFix3DPoints() const { return mFix3DPoints; }
	void setFix3DPoints(bool value) { mFix3DPoints = value; }

	const std::unordered_set<Keyframe *> getFramesToAdjust() const {return mFramesToAdjust;}
	const std::unordered_set<Feature *> getFeaturesToAdjust() const {return mFeaturesToAdjust;}

	void setMap(Map *map) {mMap=map;}
	
	void setK(const Eigen::Matrix3dr &K) { mK = K; }
	const Eigen::Matrix3dr &getK() const { return mK; }

	void setDistortion(const Eigen::Vector2d &coeff) { mParamsDistortion = coeff; }
	const Eigen::Vector2d &getDistortion() const { return mParamsDistortion; }

	void addFrameToAdjust(Keyframe &newFrame);

	bool bundleAdjust();

protected:
	float mOutlierPixelThreshold;
	float mOutlierPixelThresholdSq;
	bool mUseLocks;
	bool mFixK;
	bool mFixDistortion;
	bool mFix3DPoints;

	Map *mMap;
	std::unordered_set<Keyframe *> mFramesToAdjust;
	std::unordered_set<Feature *> mFeaturesToAdjust;

	Eigen::Vector2i mImageSize;
	Eigen::Matrix3dr mK;

	Eigen::Vector2d mParamsDistortion;
	Eigen::Vector4d mParamsK;
	std::unordered_map<Keyframe *, Eigen::Matrix<double, 1, 6>, std::hash<Keyframe*>, std::equal_to<Keyframe*>, Eigen::aligned_allocator<std::pair<Keyframe*,Eigen::Matrix<double,1,6>>>> mParamsPoses;
	std::unordered_map<Feature *, Eigen::Vector2d, std::hash<Feature*>, std::equal_to<Feature*>, Eigen::aligned_allocator<std::pair<Feature*, Eigen::Vector2d>>> mParamsFeatures;
	std::vector<FeatureMeasurement *> mMeasurementsInProblem;

	int mInlierCount;
	float mResidualsMean;
	float mResidualsStd;

	Eigen::Matrix<double, 1, 6> &getPoseParams(Keyframe *framep);
	Eigen::Vector2d &getFeatureParams(Feature *featurep);

	void getInliers();
};

}

#endif /* BUNDLEADJUSTER_H_ */
