
#ifndef PLANECALIBSYSTEM_H_
#define PLANECALIBSYSTEM_H_

#include <memory>
#include <future>
#include "stdutils.h"
#include "Map.h"

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


	void processImage(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray);
	
	//Handles thread creation and other maintenance. This should be called when idle and after processImage().
	void idle();

	void doFullBA();

protected:
	////////////////////////////////////////////////////////
	// Members
	bool mSingleThreaded;

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
