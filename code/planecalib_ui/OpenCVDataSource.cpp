/*
 * VideoImageDataSource.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: danielh
 */

#include "OpenCVDataSource.h"
#include <opencv2/imgproc.hpp>
#include "planecalib/Profiler.h"
#include "planecalib/log.h"

namespace planecalib
{

OpenCVDataSource::OpenCVDataSource(void)
{
}

OpenCVDataSource::~OpenCVDataSource(void)
{
    close();
}

bool OpenCVDataSource::open(const std::string &videoFile)
{
    if (mOpenCVCamera.isOpened())
    {
        // already opened
        return false;
    }

    if (!mOpenCVCamera.open(videoFile))
    {
        return false;
    }

	return finishOpen();
}

bool OpenCVDataSource::open(int deviceId)
{
    if (mOpenCVCamera.isOpened())
    {
        // already opened
        return false;
    }

    if (!mOpenCVCamera.open(deviceId))
    {
        return false;
    }
	//mOpenCVCamera.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
	//mOpenCVCamera.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    dropFrames(5);
    return finishOpen();
}
 bool OpenCVDataSource::finishOpen()
{
    int videoWidth = (int)mOpenCVCamera.get(cv::CAP_PROP_FRAME_WIDTH);
	int videoHeight = (int)mOpenCVCamera.get(cv::CAP_PROP_FRAME_HEIGHT);

    setSourceSize(cv::Size(videoWidth, videoHeight));

    return true;
}

void OpenCVDataSource::close(void)
{
	releaseGl();
    if (mOpenCVCamera.isOpened())
    {
        mOpenCVCamera.release();
    }
}

void OpenCVDataSource::dropFrames(int count)
{
	for(int i=0; i<count; ++i)
	{
	    cv::Mat frame;
	    mOpenCVCamera.read(frame);
	}
}

bool OpenCVDataSource::update(void)
{
    cv::Mat frame;
    if(!mOpenCVCamera.read(frame))
    {
    	return false;
    }

    assert(frame.channels()==3);
	int frameId = (int)mOpenCVCamera.get(cv::CAP_PROP_POS_FRAMES);

    // convert to RGBY format
    ImageDataSource::update(cv::Mat3b(frame), frameId);

    return true;
}

}
