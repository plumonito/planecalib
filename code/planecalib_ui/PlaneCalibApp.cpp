/*
 * SlamDriver.cpp
 *
 *  Created on: 28.2.2014
 *      Author: dan
 */

#include "PlaneCalibApp.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <GL/freeglut.h>

#include <ceres/loss_function.h>

#undef LOG
#include <glog/logging.h>
#include "planecalib/log.h"

#include "ImageDataSource.h"

//#include "planecalib/flags.h"
#include "planecalib/PlaneCalibSystem.h"
#include "planecalib/PoseTracker.h"
#include "planecalib/HomographyCalibration.h"

#include "OpenCVDataSource.h"
#include "SequenceDataSource.h"
#include "UserInterfaceInfo.h"
#include "ViewportTiler.h"

#include "flags.h"

#include "windows/MainWindow.h"
#include "windows/MapWindow.h"
#include "windows/ARWindow.h"

namespace planecalib
{

PlaneCalibApp::PlaneCalibApp()
        : mInitialized(false), mFrameCount(0), mQuit(false),
          mFrameByFrame(true), mAdvanceFrame(false), mShowProfiler(true), mShowProfilerTotals(false),
          mActiveWindow(NULL), mKeyBindings(this),
		  mRecordFrames(false)

{
}

PlaneCalibApp::~PlaneCalibApp()
{
}

// Initialization for this application
bool PlaneCalibApp::init(void)
{
    std::cout << "PlaneCalibApp init." << std::endl;
	//UserInterfaceInfo::Instance().setScreenSize(cv::Size2i(System::ScreenGetWidth(), System::ScreenGetHeight()));

#ifdef ENABLE_LOG
	std::cout << "Logging is enabled." << std::endl;
#else
	std::cout << "Logging is disabled." << std::endl;
#endif

    Profiler::Instance().setCurrentThreadName("render");
    char glogstr[] =
    //"planecalib";
            "";//Empty str disables the creation  of a log file from glog. Only ceres uses glog. Not needed.


    google::InitGoogleLogging(glogstr);

    if (!initImageSrc())
    {
		MYAPP_LOG << "Couldn't initialize image source.\n";
        return false;
    }

    // Initialize Shader
    if (!mShaders.init())
    {
        return false;
    }

	//Determine downscale at input
	int width = mImageSrc->getSourceSize().width;
	mDownsampleInputCount = 0;
	//while(width > FLAGS_DriverMaxImageWidth)
	//{
	//	width = (width+1)/2;
	//	mDownsampleInputCount++;
	//}
	int scale = 1<<mDownsampleInputCount;

	mImageSrc->setDownsample(mDownsampleInputCount);
	mImageSize = eutils::FromSize(mImageSrc->getSize());
	MYAPP_LOG << "Input image size after downsampling: " << eutils::ToSize(mImageSize) << "\n";

	//Get first frame
	if(!mImageSrc->update())
    {
		MYAPP_LOG << "Couldn't get first frame from image source.\n";
    	return false;
    }

	//System
	cv::Mat1b imageGray = mImageSrc->getImgGray();
	cv::Mat3b imageColor = mImageSrc->getImgColor();
	mSystem.reset( new PlaneCalibSystem() );
	mSystem->init(mImageSrc->getCaptureTime(), imageColor, imageGray);
	mSystem->setSingleThreaded(FLAGS_DriverSingleThreaded);

	//Add windows
	mWindows.push_back(std::unique_ptr<BaseWindow>(new MainWindow()));
	mWindows.push_back(std::unique_ptr<BaseWindow>(new MapWindow()));
	mWindows.push_back(std::unique_ptr<BaseWindow>(new ARWindow()));

	//Add bindings
	mKeyBindings.addBinding(true, GLUT_KEY_F5, static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::runVideo), "Run the video stream.");
	mKeyBindings.addBinding(true, GLUT_KEY_F8, static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::saveMap), "Save the map to disk.");
	mKeyBindings.addBinding(true, GLUT_KEY_F9, static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::loadMap), "Load the map from disk.");
	mKeyBindings.addBinding(false, ' ', static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::stepVideo), "Advance one frame.");
	mKeyBindings.addBinding(false, 'p', static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::toggleProfilerMode), "Toggle profiler mode.");
	mKeyBindings.addBinding(false, 'P', static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::resetProfiler), "Reset profiler counts.");
	mKeyBindings.addBinding(false, 'r', static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::resetSystem), "Reset the slam system.");
	mKeyBindings.addBinding(false, 'c', static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::recordOneFrame), "Record a single frame.");
	mKeyBindings.addBinding(false, 'R', static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::startRecording), "Reset and start recording.");

	for(int i=0; i<(int)mWindows.size(); ++i)
		mKeyBindings.addBinding(false, i + '1', static_cast<KeyBindingHandler<PlaneCalibApp>::BindingFunc>(&PlaneCalibApp::changeWindowKey), "Show window: " + mWindows[i]->getName());

	mKeyBindings.addBinding(false, 27, static_cast<KeyBindingHandler<PlaneCalibApp>::SimpleBindingFunc>(&PlaneCalibApp::escapePressed), "Quit.");

	MYAPP_LOG << "\nBasic keys:\n";
	mKeyBindings.showHelp();

	setActiveWindow(mWindows[0].get());

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);

	mFPS = 0;
	mLastFPSCheck = std::chrono::high_resolution_clock::now();
	mFPSUpdateDuration = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::seconds(1));
	mFPSSampleAccum = std::chrono::high_resolution_clock::duration(0);
	mFPSSampleCount = 0;

	//Record vars
	mRecordId = 0;
	mRecordFileFormat = FLAGS_DriverRecordPath + "frame%.4d.jpg";

	mInitialized = true;
    return true;
}

void PlaneCalibApp::resetSystem()
{
	cv::Mat1b imageGray = mImageSrc->getImgGray();
	cv::Mat3b imageColor = mImageSrc->getImgColor();
	mSystem->init(mImageSrc->getCaptureTime(), imageColor, imageGray);

	for(int i=0; i<(int)mWindows.size(); ++i)
		mWindows[i]->requireInit();

	if(mActiveWindow)
	{
		mActiveWindow->init(this, mImageSize);
		mActiveWindow->updateState();
	}
}

void PlaneCalibApp::startRecording()
{
	resetSystem();
	mRecordFrames = true;
	mRecordId = 0;
	mRecordFileFormat = FLAGS_DriverRecordPath + "frame%.4d.jpg";

	//Delete all previous
	bool filesToDelete=true;
	int deleteId=0;
	char buffer[1024];
	while(filesToDelete)
	{
		sprintf(buffer, mRecordFileFormat.c_str(), deleteId++);
		if(std::remove(buffer))
			filesToDelete = false;
	};
}

void PlaneCalibApp::recordFrame(cv::Mat3b &im)
{
	//Convert to bgr
	cv::Mat3b bgr;
	cv::cvtColor(im, bgr, cv::COLOR_RGB2BGR);

	//Save
	char buffer[1024];
	sprintf(buffer, mRecordFileFormat.c_str(), mRecordId++);

	cv::imwrite(buffer, bgr);
}

void PlaneCalibApp::setActiveWindow(BaseWindow *window)
{
	if(!window->isInitialized())
	{
		window->init(this, mImageSize);
		window->setCurrentImageTexture(mImageSrc->getTextureTarget(), mImageSrc->getTextureId());
		window->showHelp();
	}
	else
	{
		window->resize();
	}
	window->updateState();
	mActiveWindow = window;
}

void PlaneCalibApp::resize()
{
	mActiveWindow->resize();
}

bool PlaneCalibApp::initImageSrc()
{
	mUsingCamera = false;
	if(!FLAGS_DriverVideoFile.empty())
	{
		//Use video file
	    std::string videoFilename = FLAGS_DriverDataPath + "/" + FLAGS_DriverVideoFile;

		MYAPP_LOG << "Video file: " << videoFilename << "\n";
		OpenCVDataSource *source = new OpenCVDataSource();
	    mImageSrc.reset(source);
	    if(!source->open(videoFilename))
	    {
			MYAPP_LOG << "Error opening video.\n";
	        return NULL;
	    }

		MYAPP_LOG << "Opened video file succesfully\n";
	}
	else if(!FLAGS_DriverSequenceFormat.empty())
	{
		//Use image sequence
	    std::string sequence = FLAGS_DriverDataPath + "/" + FLAGS_DriverSequenceFormat;

		MYAPP_LOG << "Image sequence: " << sequence << "\n";
		SequenceDataSource *source = new SequenceDataSource();
	    mImageSrc.reset(source);
	    if(!source->open(sequence, FLAGS_DriverSequenceStartIdx))
	    {
			MYAPP_LOG << "Error opening sequence.\n";
	        return NULL;
	    }

		MYAPP_LOG << "Opened image sequence succesfully\n";
	}
	else
	{
		//Use camera
		OpenCVDataSource *source = new OpenCVDataSource();
	    mImageSrc.reset(source);
	    if(!source->open(FLAGS_DriverCameraId))
	    {
			MYAPP_LOG << "Error opening camera.\n";
	        return false;
	    }

	    mUsingCamera = true;
		MYAPP_LOG << "Camera opened succesfully\n";
	}

	MYAPP_LOG << "Image source size: " << mImageSrc->getSourceSize() << "\n";

	return true;
}

void PlaneCalibApp::exit()
{
	MYAPP_LOG << "clean up...\n";
    mShaders.free();
}

void PlaneCalibApp::keyDown(bool isSpecial, uchar key)
{
	if(mKeyBindings.dispatchKeyDown(isSpecial, key))
		return;
	mActiveWindow->keyDown(isSpecial, key);
}

void PlaneCalibApp::keyUp(bool isSpecial, uchar key)
{
	if(mKeyBindings.dispatchKeyUp(isSpecial, key))
		return;
	mActiveWindow->keyUp(isSpecial, key);
}

void PlaneCalibApp::toggleProfilerMode()
{
	if(!mShowProfiler)
		mShowProfiler = true;
	else if(!mShowProfilerTotals)
		mShowProfilerTotals = true;
	else
		mShowProfiler = mShowProfilerTotals = false;
Profiler::Instance().setShowTotals(mShowProfilerTotals);
}

void PlaneCalibApp::changeWindowKey(bool isSpecial, unsigned char key)
{
	int idx = (key-'0')-1;
	if(idx < 0)
		idx = 10;
	if(idx >= (int)mWindows.size())
		idx = mWindows.size()-1;
	setActiveWindow(mWindows[idx].get());
}

void PlaneCalibApp::touchDown(int id, int x, int y)
{
	mActiveWindow->touchDown(id, x, y);
}

void PlaneCalibApp::touchMove(int x, int y)
{
	mActiveWindow->touchMove(x, y);
}

void PlaneCalibApp::touchUp(int id, int x, int y)
{
	mActiveWindow->touchUp(id, x, y);
}

// Main draw function
void PlaneCalibApp::draw(void)
{
    if(!mFrameByFrame || mAdvanceFrame)
	{
        bool isFrameAvailable;
        {
        	ProfileSection section("updateFrame");

        	//Drop frames
        	mImageSrc->dropFrames(FLAGS_DriverDropFrames);

			isFrameAvailable = mImageSrc->update();
        }
        if(isFrameAvailable)
		{
            ProfileSection section("execute");

			mFrameCount++;
			mAdvanceFrame = false;

			//MYAPP_LOG << "\nFrame #" << mFrameCount << "\n";

			//Read new input frame
			cv::Mat1b imageGray = mImageSrc->getImgGray();
			cv::Mat3b imageColor = mImageSrc->getImgColor();

			//Record
			if (mRecordFrames || mRecordOneFrame)
			{
				mRecordOneFrame = false;
				recordFrame(imageColor);
			}

			//Process new frame
			auto tic = std::chrono::high_resolution_clock::now();
			mSystem->processImage(mImageSrc->getCaptureTime(), imageColor, imageGray);
			mFPSSampleAccum += std::chrono::high_resolution_clock::now()-tic;
			mFPSSampleCount++;

			mActiveWindow->updateState();

			//if (mSystem->getTracker().isLost())
			//	mFrameByFrame = true;
		}
	}

	//mSlam.idle();

    {
        ProfileSection section("draw");

		glClearColor(1.0, 1.0, 1.0, 1.0);
		//glClearColor(0.0, 0.0, 0.0, 0.0);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        {
        	ProfileSection section("windowDraw");
        	mActiveWindow->draw();
        }

	    //Text
	    std::stringstream ss;
		ss << "FPS " << (int)mFPS << "\n";
		ss << "Frame " << mFrameCount << "\n";

	    if(mShowProfiler)
		{
		    Profiler::Instance().logStats(ss);
		}

	    Eigen::Vector2i screenSize = UserInterfaceInfo::Instance().getScreenSize();
		float viewportAspect = static_cast<float>(screenSize.x()) / screenSize.y();
		glViewport(0,0,screenSize.x(),screenSize.y());
		mShaders.getText().setMVPMatrix(ViewportTiler::GetImageSpaceMvp(viewportAspect, screenSize));

	    mShaders.getText().setActiveFontSmall();
	    mShaders.getText().setRenderCharHeight(10);
	    mShaders.getText().setCaret(Eigen::Vector2f(0,0));
	    mShaders.getText().setColor(StaticColors::Green());
	    mShaders.getText().renderText(ss);

	    //Map stats in the bottom
		const float kMapStatsFontHeight = 10.0f;
		std::vector<Eigen::Vector2f> corners;
		corners.push_back(Eigen::Vector2f(0.0f, (float)screenSize.y() - 2 * kMapStatsFontHeight));
		corners.push_back(Eigen::Vector2f((float)screenSize.x(), (float)screenSize.y() - 2 * kMapStatsFontHeight));
		corners.push_back(Eigen::Vector2f(0.0f, (float)screenSize.y()));
		corners.push_back(Eigen::Vector2f((float)screenSize.x(), (float)screenSize.y()));
	    mShaders.getColor().setMVPMatrix(ViewportTiler::GetImageSpaceMvp(viewportAspect, screenSize));
	    mShaders.getColor().drawVertices(GL_TRIANGLE_STRIP, corners.data(), 4, StaticColors::Black(0.5f));

		mShaders.getText().setRenderCharHeight(kMapStatsFontHeight);
		mShaders.getText().setCaret((corners[0] + Eigen::Vector2f(kMapStatsFontHeight / 2, kMapStatsFontHeight / 2)).eval());
		mShaders.getText().setColor(StaticColors::White());
		{
			TextRendererStream ts(mShaders.getText());

			int frameCount = mSystem->getMap().getKeyframes().size();
			int count3D = mSystem->getMap().getFeatures().size();
		//	if (!FLAGS_DisableRegions)
		//		ts << "Regions: " << mSlam.getMap().getRegions().size() << ", ";
			ts << "Keyframes: " << frameCount << ", Features: " << count3D << "";

		//	switch (mSlam.getTracker().getPoseType() )
		//	{
		//		case EPoseEstimationType::PureRotation:
		//			ts.setColor(StaticColors::Yellow());
		//			ts << " PURE ROTATION";
		//			break;
		//		case EPoseEstimationType::Essential:
		//			ts.setColor(StaticColors::Green());
		//			ts << " ESSENTIAL MODEL";
		//			break;
		//		case EPoseEstimationType::Invalid:
		//			ts.setColor(StaticColors::Red());
		//			ts << " LOST";
		//			break;
		//	}
		//	ts.setColor(StaticColors::White());

		//	//Expander status
		//	switch (mSlam.getMapExpander().getStatus())
		//	{
		//	case ESlamMapExpanderStatus::CheckingFrame:
		//		ts << ", expander checking";
		//		break;
		//	case ESlamMapExpanderStatus::AddingFrame:
		//		ts << ", expander adding";
		//		break;
		//	case ESlamMapExpanderStatus::SingleFrameBA:
		//		ts << ", expander one-frame BA";
		//		break;
		//	}

		//	//BA status
		//	if (mSlam.isBARunning())
		//	{
		//		ts << ", ";
		//		if (mSlam.getActiveRegion()->getAbortBA())
		//		{
		//			ts.setColor(StaticColors::Yellow());
		//			ts << "aborting BA";
		//			ts.setColor(StaticColors::White());
		//		}
		//		else
		//		{
		//			ts << "BA is running";
		//		}
		//	}
		//	else if (mSlam.getActiveRegion()->getShouldBundleAdjust())
		//	{
		//		ts << ", ";
		//		ts.setColor(StaticColors::Yellow());
		//		ts << "BA pending";
		//		ts.setColor(StaticColors::White());
		//	}
		}
    }

	//Update FPS
	auto now = std::chrono::high_resolution_clock::now();
	auto elapsedDuration = now - mLastFPSCheck;
	if (elapsedDuration > mFPSUpdateDuration)
	{
		if (mFPSSampleCount)
			mFPS = mFPSSampleCount / std::chrono::duration_cast<std::chrono::duration<float>>(mFPSSampleAccum).count();
		
		mFPSSampleCount = 0;
		mFPSSampleAccum = std::chrono::high_resolution_clock::duration(0);

		mLastFPSCheck = now;
	}
}

void PlaneCalibApp::saveMap()
{
	//Serializer s;
	//s.open("save/", "map.yml");
	//s.addObject(&mSlam.getMap());
	//s.serializeAll();
}

void PlaneCalibApp::loadMap()
{
	//Deserializer s;
	//s.open("save/", "map.yml");
	//s.deserialize();

	//mCamera = s.getObjectForOwner<CameraModel>();
	//std::unique_ptr<SlamMap> map = s.getObjectForOwner<SlamMap>();
	//mSlam.init(mCamera.get(), std::move(map));
}

}
