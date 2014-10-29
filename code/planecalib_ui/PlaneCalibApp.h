
#ifndef PLANECALIBAPP_H_
#define PLANECALIBAPP_H_

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <opencv2/core/core.hpp>

#include "planecalib/Profiler.h"

#include "Application.h"
#include "shaders/DTSlamShaders.h"
#include "windows/BaseWindow.h"

namespace planecalib
{
class ImageDataSource;
class OpenCVDataSource;

class PlaneCalibApp: public Application
{
private:
	bool mInitialized;

    int mFrameCount;

    bool mUsingCamera;
    std::unique_ptr<ImageDataSource> mImageSrc;
    int mDownsampleInputCount;
    cv::Size2i mImageSize;

    DTSlamShaders mShaders;

    volatile bool mQuit;

    bool mFrameByFrame;
    bool mAdvanceFrame;

    bool mRecordFrames;
    int mRecordId;
    std::string mRecordFileFormat;

    bool mShowProfiler;
    bool mShowProfilerTotals;
	
	float mFPS;
	std::chrono::high_resolution_clock::time_point mLastFPSCheck;
	std::chrono::high_resolution_clock::duration mFPSUpdateDuration;
	std::chrono::high_resolution_clock::duration mFPSSampleAccum;
	int mFPSSampleCount;

    //SlamSystem mSlam;

    std::vector<std::unique_ptr<BaseWindow>> mWindows;
    BaseWindow *mActiveWindow;

public:
	PlaneCalibApp();
	~PlaneCalibApp();

    bool getFinished() {return mQuit;}

    DTSlamShaders &getShaders() {return mShaders;}

    bool init();
    void resize();
    void exit();
    bool loop() {return mQuit;}

    void keyDown(bool isSpecial, uchar key);
    void keyUp(bool isSpecial, uchar key);
    void touchDown(int id, int x, int y);
    void touchMove(int x, int y);
    void touchUp(int id, int x, int y);

    void draw(void);

private:
    bool initImageSrc();

	KeyBindingHandler<PlaneCalibApp> mKeyBindings;
    void runVideo() {mFrameByFrame = !mFrameByFrame; mAdvanceFrame = false;}
    void stepVideo(){mFrameByFrame = true; mAdvanceFrame = true;}
    void toggleProfilerMode();
    void resetProfiler() {Profiler::Instance().reset();}
    void escapePressed() {mQuit=true;}
    void changeWindowKey(bool isSpecial, unsigned char key);
    void resetSystem();
    void startRecording();
    void recordFrame(cv::Mat3b &im);

	void saveMap();
	void loadMap();

    void setActiveWindow(BaseWindow *window);
};

}

#endif /* SLAMDRIVER_H_ */
