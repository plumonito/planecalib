#ifndef PNPESTIMATION_H_
#define PNPESTIMATION_H_

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "CameraModel.h"
#include "Map.h"
#include "FeatureMatcher.h"
#include "cvutils.h"
#include "PoseEstimationCommon.h"

namespace planecalib {

class PoseReprojectionError3D;
class PoseReprojectionError2D;
class EpipolarSegmentErrorForPose;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PnPRefiner
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PnPRefiner
{
public:
	PnPRefiner()		
	{
	}

	void setCamera(const CameraModel *camera) { mCamera = camera; }
	void setOutlierThreshold(float pixelThreshold)
	{
		mOutlierPixelThreshold = pixelThreshold;
		mOutlierPixelThresholdSq = mOutlierPixelThreshold*mOutlierPixelThreshold;
	}


	bool getReprojectionErrors(const FeatureMatch &match,
		const Eigen::Matrix3dr &R,
		const Eigen::Vector3d &translation,
		MatchReprojectionErrors &errors);

	void getInliers(const std::vector<FeatureMatch> &matches,
			const Eigen::Matrix3dr &R,
			const Eigen::Vector3d &translation,
			int &inlierCount,
			std::vector<MatchReprojectionErrors> &errors);

	void refinePose(const std::vector<FeatureMatch> &matches,
			Eigen::Matrix3fr &rotation,
			Eigen::Vector3f &translation,
			int &inlierCount,
			std::vector<MatchReprojectionErrors> &errors);

protected:
	const CameraModel *mCamera;
	float mOutlierPixelThreshold;
	float mOutlierPixelThresholdSq;
};

}

#endif /* PNPESTIMATION_H_ */
