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
#include "CameraModel.h"

namespace planecalib {

class PoseTracker;

class BundleAdjuster
{
public:
	typedef CameraModel::TDistortionModel::TParamVector TDistortionParamVector;

	BundleAdjuster():
		mUseLocks(true), mOnlyDistortion(false), mParamsDistortion(TDistortionParamVector::Zero()), mOutlierPixelThreshold(3), mOutlierPixelThresholdSq(9)
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
	void setCamera(CameraModel *camera);

	void addFrameToAdjust(Keyframe &newFrame);
	const std::unordered_set<Keyframe *> getFramesToAdjust() const { return mFramesToAdjust; }
	const std::unordered_set<Feature *> getFeaturesToAdjust() const {return mFeaturesToAdjust;}

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

	CameraModel *mCamera;

	Eigen::Vector2d mParamsPrincipalPoint;
	TDistortionParamVector mParamsDistortion;
	std::vector<FeatureMeasurement *> mMeasurementsInProblem;

	bool isInlier(const FeatureMeasurement &measurement);

	void getInliers(int &inlierCount, std::vector<FeatureMeasurement *> &outliers);
};

}

#endif /* BUNDLEADJUSTER_H_ */
