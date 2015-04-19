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
	SceneGenerator() : mNoiseStd(1), mCamera(NULL), mLogScene(false), mVerbose(false)
	{}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CameraModel *getCamera() const { return mCamera; }
	void setCamera(CameraModel *value) { mCamera = value; }

	float getNoiseStd() const { return mNoiseStd; }
	void setNoiseStd(float value) 
	{ 
		mNoiseStd = value; 
		if (mNoiseStd <= 0) 
			mNoiseStd = std::numeric_limits<float>::epsilon(); 
	}

	std::unique_ptr<Map> generateSyntheticMap();
	std::unique_ptr<Map> generateSyntheticMap(float normalAngle);
	std::unique_ptr<Map> generateRandomMap(int frameCount);
	std::unique_ptr<Map> generateRandomMap(int frameCount, float normalAngle);
	std::unique_ptr<Map> generateVariableNormal(float normalAngle);

	bool mLogScene;
	bool mVerbose;

protected:
	std::random_device mRandomDevice;
	CameraModel *mCamera;
	float mNoiseStd;

	void generateSyntheticPoses(std::vector<Eigen::Matrix3fr> &posesR, std::vector<Eigen::Vector3f> &posesCenter);
	void generateRandomPoses(int frameCount, std::vector<Eigen::Matrix3fr> &posesR, std::vector<Eigen::Vector3f> &posesCenter);
	
	void generateRandomRef(float normalAngle, Eigen::Matrix3fr &R, Eigen::Vector3f &center);

	std::unique_ptr<Map> generateFromPoses(const std::vector<Eigen::Matrix3fr> &posesR, const std::vector<Eigen::Vector3f> &posesCenter);
};

}

#endif 
