#ifndef SCENEGENERATOR_H_
#define SCENEGENERATOR_H_

#include <Eigen/Dense>
#include <random>
#include <memory>
#include "planecalib/eutils.h"
#include "planecalib/CameraModel.h"

namespace planecalib
{

class PlaneCalibSystem;
class Map;
class FrameTrackingData;
class PoseTracker;

class SceneGenerator
{
public:
	SceneGenerator() : mNoiseStd(1)
	{}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	float getNoiseStd() const { return mNoiseStd; }
	void setNoiseStd(float value) 
	{ 
		mNoiseStd = value; 
		if (mNoiseStd <= 0) 
			mNoiseStd = std::numeric_limits<float>::epsilon(); 
	}

	std::unique_ptr<Map> generateSyntheticMap(const CameraModel &camera);
	std::unique_ptr<Map> generateRandomPoses(const CameraModel &camera, int frameCount);

protected:
	std::random_device mRandomDevice;
	float mNoiseStd;

	std::unique_ptr<Map> generateFromPoses(const CameraModel &camera, const std::vector<Eigen::Matrix3fr> &posesR, const std::vector<Eigen::Vector3f> &posesCenter);
};

}

#endif 
