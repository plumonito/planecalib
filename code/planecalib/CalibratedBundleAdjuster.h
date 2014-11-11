
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
};

}

#endif /* BUNDLEADJUSTER_H_ */
