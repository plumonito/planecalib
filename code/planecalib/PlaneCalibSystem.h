
#ifndef PLANECALIBSYSTEM_H_
#define PLANECALIBSYSTEM_H_

#include <memory>
#include <future>
#include "stdutils.h"
#include "Map.h"
#include "PoseTracker.h"
//#include "SlamMapExpander.h"

namespace planecalib
{

class PlaneCalibSystem
{
public:
	PlaneCalibSystem() : mSingleThreaded(false) {}

	bool init(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray);

	bool isSingleThreaded() const {return mSingleThreaded;}
	void setSingleThreaded(bool value) { mSingleThreaded = value; MYAPP_LOG << "Set Slam single threaded = " << mSingleThreaded << "\n"; }

	bool isExpanderRunning() {return mExpanderFuture.valid() && !mExpanderFinished;}

	Map &getMap() {return *mMap;}
	PoseTracker &getTracker() {return *mTracker;}
	//SlamMapExpander &getMapExpander() {return *mMapExpander;}

	void processImage(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray);
	
	//Handles thread creation and other maintenance. This should be called when idle and after processImage().
	void idle();

	//Debug
	std::vector<Eigen::Matrix3fr> mAllH;
	Eigen::Matrix3fr mK;
	Eigen::Vector3d mNormal;

protected:
	////////////////////////////////////////////////////////
	// Members
	bool mSingleThreaded;

	std::unique_ptr<Map> mMap;

	std::unique_ptr<PoseTracker> mTracker;
	//std::unique_ptr<SlamMapExpander> mMapExpander;

	std::atomic<bool> mExpanderCheckPending;

	std::future<Keyframe *> mExpanderFuture;
	std::atomic<bool> mExpanderFinished;
	//std::atomic<bool> mExpanderAdding;

	//std::future<void> mBAFuture;
	//std::atomic<bool> mBAFinished;

	////////////////////////////////////////////////////////
	// Methods

	//std::unique_ptr<SlamMapExpander::CheckData> createDataForExpander();

	//static Keyframe *ExpanderTask(PlaneCalibSystem *system, MapExpander::CheckData *dataPtr, bool useLocks);
	//Keyframe *expanderTask(std::unique_ptr<MapExpander::CheckData> data, bool useLocks);

public:
};

} 

#endif /* SLAMSYSTEM_H_ */
