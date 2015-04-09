
#ifndef PLANECALIBSYSTEM_H_
#define PLANECALIBSYSTEM_H_

#include <memory>
#include <future>
#include "stdutils.h"
#include "Map.h"
#include "CameraDistortionModel.h"

namespace planecalib
{

class PoseTracker;
class HomographyCalibration;

class PlaneCalibSystem
{
public:
	PlaneCalibSystem() : mSingleThreaded(false) {}
	~PlaneCalibSystem();

	bool init(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray);

	bool isSingleThreaded() const {return mSingleThreaded;}
	void setSingleThreaded(bool value) { mSingleThreaded = value; MYAPP_LOG << "Set Slam single threaded = " << mSingleThreaded << "\n"; }

	bool isExpanderRunning() {return mExpanderFuture.valid() && !mExpanderFinished;}

	Map &getMap() {return *mMap;}
	void setMap(std::unique_ptr<Map> map);

	PoseTracker &getTracker() { return *mTracker; }
	
	HomographyCalibration &getCalib() { return *mCalib; }
	//SlamMapExpander &getMapExpander() {return *mMapExpander;}

	const Eigen::Matrix3fr &getK() const { return mK; }
	const Eigen::Vector3f &getNormal() const { return mNormal; }
	const RadialCameraDistortionModel &getDistortion() const { return *mActiveDistortion; }


	void processImage(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray);
	
	//Handles thread creation and other maintenance. This should be called when idle and after processImage().
	void idle();

	void doHomographyBA();
	void doHomographyCalib();
	void doFullBA();

	//Testing
	void generateSyntheticMap(const Eigen::Matrix3fr &k, const Eigen::Vector2f &distortion, float measurementNoiseVar);

protected:
	////////////////////////////////////////////////////////
	// Members
	bool mSingleThreaded;

	Eigen::Vector2f mHomographyP0;
	RadialCameraDistortionModel mHomographyDistortion;	
	RadialCameraDistortionModel mHomographyDistortionInv;
	RadialCameraDistortionModel mCameraDistortion;
	RadialCameraDistortionModel *mActiveDistortion;

	Eigen::Vector3f mNormal;
	Eigen::Matrix3fr mK;

	std::unique_ptr<Map> mMap;

	std::unique_ptr<PoseTracker> mTracker;
	//std::unique_ptr<SlamMapExpander> mMapExpander;
	std::unique_ptr<HomographyCalibration> mCalib;

	std::atomic<bool> mExpanderCheckPending;

	std::future<Keyframe *> mExpanderFuture;
	std::atomic<bool> mExpanderFinished;
	//std::atomic<bool> mExpanderAdding;

	//std::future<void> mBAFuture;
	//std::atomic<bool> mBAFinished;

	////////////////////////////////////////////////////////
	// Methods

	void createKeyframe();
	//std::unique_ptr<SlamMapExpander::CheckData> createDataForExpander();

	//static Keyframe *ExpanderTask(PlaneCalibSystem *system, MapExpander::CheckData *dataPtr, bool useLocks);
	//Keyframe *expanderTask(std::unique_ptr<MapExpander::CheckData> data, bool useLocks);

public:
};

} 

#endif /* SLAMSYSTEM_H_ */
