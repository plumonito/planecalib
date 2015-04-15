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
	SceneGenerator()
	{}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	std::unique_ptr<Map> generateSyntheticMap(const CameraModel &camera, float measurementNoiseStd);

protected:
	std::random_device mRandomDevice;
};

}

#endif 
