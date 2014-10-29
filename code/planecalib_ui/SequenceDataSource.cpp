/*
 * SequenceDataSource.cpp
 *
 *  Created on: 22.3.2014
 *      Author: dan
 */

#include "SequenceDataSource.h"
#include <opencv2/imgproc.hpp>
#include "planecalib/Profiler.h"
#include "planecalib/log.h"

namespace planecalib
{

SequenceDataSource::SequenceDataSource(void)
{
}

SequenceDataSource::~SequenceDataSource(void)
{
    close();
}

cv::Mat SequenceDataSource::readImage(int idx)
{
	char buffer[1024];
	sprintf(buffer, mSequenceFormat.c_str(), idx);

	try
	{
	return cv::imread(buffer, cv::IMREAD_COLOR);
	}
	catch(cv::Exception &ex)
	{
		DTSLAM_LOG << "\n\nAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH\n\n";
		throw ex;
		return cv::Mat();
	}
}

bool SequenceDataSource::open(const std::string &sequenceFormat, int startIdx)
{
	mSequenceFormat = sequenceFormat;
	mCurrentFrameIdx = startIdx;

	cv::Mat sampleImg = readImage(startIdx);
	if(sampleImg.empty())
	{
		DTSLAM_LOG << "Error opening image sequence, format=" << sequenceFormat << ", startIdx=" << startIdx << "\n";
		return false;
	}
    setSourceSize(sampleImg.size());
    return true;
}

void SequenceDataSource::close(void)
{
	releaseGl();
}

void SequenceDataSource::dropFrames(int count)
{
	mCurrentFrameIdx += count;
}

bool SequenceDataSource::update(void)
{
    cv::Mat frame = readImage(mCurrentFrameIdx);

    if(frame.empty())
    {
    	return false;
    }

    assert(frame.channels()==3);

    ImageDataSource::update(cv::Mat3b(frame), mCurrentFrameIdx);

    mCurrentFrameIdx++;

    return true;
}

} /* namespace planecalib */
