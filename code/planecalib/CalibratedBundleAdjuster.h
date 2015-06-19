
#ifndef CALIBRATEDBUNDLEADJUSTER_H_
#define CALIBRATEDBUNDLEADJUSTER_H_

#include "eutils.h"
#include <vector>
#include <unordered_set>
#include <memory>
#include <opencv2/core.hpp>
#include "Map.h"
#include "CameraModel.h"

namespace planecalib {

class PoseTracker;

class CalibratedBundleAdjuster
{
public:
	CalibratedBundleAdjuster() :
		mUseLocks(true), mFixPrincipalPoint(false), mFixDistortion(false), mFixFocalLengths(false), mFix3DPoints(false), mPrincipalPoint(Eigen::Vector2d::Zero()), mParamsDistortion(TDistortionParamsVector::Zero()), mFocalLengths(Eigen::Vector2d::Zero()), mOutlierPixelThreshold(3), mOutlierPixelThresholdSq(9)
	{}

	void setOutlierThreshold(float pixelThreshold)
	{
		mOutlierPixelThreshold = pixelThreshold;
		mOutlierPixelThresholdSq = pixelThreshold*pixelThreshold;
	}

	bool getUseLocks() const {return mUseLocks;}
	void setUseLocks(bool value) {mUseLocks = value;}

	bool getFixPrincipalPoint() const { return mFixPrincipalPoint; }
	void setFixPrincipalPoint(bool value) { mFixPrincipalPoint = value; }

	bool getFixDistortion() const { return mFixDistortion; }
	void setFixDistortion(bool value) { mFixDistortion = value; }

	bool getFixFocalLengths() const { return mFixFocalLengths; }
	void setFixFocalLengths(bool value) { mFixFocalLengths = value; }

	bool getFix3DPoints() const { return mFix3DPoints; }
	void setFix3DPoints(bool value) { mFix3DPoints = value; }

	const std::unordered_set<Keyframe *> getFramesToAdjust() const {return mFramesToAdjust;}
	const std::unordered_set<Feature *> getFeaturesToAdjust() const {return mFeaturesToAdjust;}

	void setMap(Map *map) {mMap=map;}
	
	void setCamera(CameraModel *camera);

	void addFrameToAdjust(Keyframe &newFrame);

	bool bundleAdjust();

protected:
	typedef CameraModel::TDistortionModel::TParamVector TDistortionParamsVector;

	float mOutlierPixelThreshold;
	float mOutlierPixelThresholdSq;
	bool mUseLocks;
	bool mFixPrincipalPoint;
	bool mFixDistortion;
	bool mFixFocalLengths;
	bool mFix3DPoints;

	Map *mMap;
	std::unordered_set<Keyframe *> mFramesToAdjust;
	std::unordered_set<Feature *> mFeaturesToAdjust;

	CameraModel *mCamera; //Will be updated at the end of the BA

	Eigen::Vector4d mPrincipalPoint;
	TDistortionParamsVector mParamsDistortion;
	Eigen::Vector4d mFocalLengths;

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
