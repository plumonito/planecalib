
#ifndef PLANECALIBSYSTEM_H_
#define PLANECALIBSYSTEM_H_

#include <memory>
#include <future>
#include "stdutils.h"
#include "Map.h"
#include "CameraModel.h"

namespace planecalib
{

class PoseTracker;
class HomographyCalibration;

class PlaneCalibSystem
{
public:
	PlaneCalibSystem() : mSingleThreaded(false), mExpectedPixelNoiseStd(1), mUse3DGroundTruth(false), mFix3DPoints(false), mUseNormalizedConstraints(true), mSuccesfulTrackCount(0)
	{}
	~PlaneCalibSystem();

	bool init(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray);

	bool isSingleThreaded() const {return mSingleThreaded;}
	void setSingleThreaded(bool value) { mSingleThreaded = value; MYAPP_LOG << "Set Slam single threaded = " << mSingleThreaded << "\n"; }

	bool isExpanderRunning() {return mExpanderFuture.valid() && !mExpanderFinished;}

	float getExpectedPixelNoiseStd() { return mExpectedPixelNoiseStd; }
	void setExpectedPixelNoiseStd(float value) { mExpectedPixelNoiseStd = value; }

	Map &getMap() {return *mMap;}
	void setMap(std::unique_ptr<Map> map);

	const CameraModel &getCamera() const { return mCamera; }
	void setCamera(const CameraModel &value) { mCamera = value; }

	void resetCalib();

	bool getUse3DGroundTruth()  const { return mUse3DGroundTruth; }
	void setUse3DGroundTruth(bool value)  { mUse3DGroundTruth = value; }

	bool getFix3DPoints()  const { return mFix3DPoints; }
	void setFix3DPoints(bool value)  { mFix3DPoints  = value; }

	bool getUseNormalizedConstraints()  const { return mUseNormalizedConstraints; }
	void setUseNormalizedConstraints(bool value)  { mUseNormalizedConstraints = value; }

	PoseTracker &getTracker() { return *mTracker; }
	
	HomographyCalibration &getCalib() { return *mCalib; }
	
	const Eigen::Vector3f &getNormal() const { return mNormal; }

	void processImage(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray);
	
	//Handles thread creation and other maintenance. This should be called when idle and after processImage().
	void idle();

	void doHomographyBA();
	void doHomographyCalib(bool fixP0);
	void doFullBA();
	void doValidationBA();

	bool mKeyframeAdded;

protected:
	////////////////////////////////////////////////////////
	// Members
	bool mSingleThreaded;
	bool mUse3DGroundTruth;
	bool mFix3DPoints;
	bool mUseNormalizedConstraints;

	float mExpectedPixelNoiseStd;

	Eigen::Vector2f mPrincipalPoint;
	DivisionDistortionModel mDistortion;	
	Eigen::Vector2f mFocalLengths;

	Eigen::Vector3f mNormal;

	CameraModel mCamera;

	std::unique_ptr<Map> mMap;

	std::unique_ptr<PoseTracker> mTracker;
	int mSuccesfulTrackCount;

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
