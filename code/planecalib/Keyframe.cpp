/*
 * SlamKeyFrame.cpp
 *
 *  Created on: 8.2.2014
 *      Author: dan
 */

#include "Keyframe.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "Profiler.h"
#include "Map.h"
#include "flags.h"
#include "cvutils.h"

namespace planecalib {

Keyframe::Keyframe() :
		mKeypoints(new std::vector<std::vector<KeypointData>>())
{
}

Keyframe::Keyframe(const Keyframe &copyFrom) :
		mTimestamp(copyFrom.mTimestamp), mSBI(copyFrom.mSBI), mSBIdx(copyFrom.mSBIdx), mSBIdy(copyFrom.mSBIdy),
		mColorImage(copyFrom.mColorImage), mPyramid(copyFrom.mPyramid), mKeypoints(copyFrom.mKeypoints),
		mPose(copyFrom.mPose), mMeasurements(copyFrom.mMeasurements)
{
}

Keyframe::~Keyframe()
{
}

void Keyframe::init(const cv::Mat3b &imageColor, const cv::Mat1b &imageGray)
{
	mColorImage = imageColor;

	mPyramid.create(imageGray, FLAGS_PyramidMaxTopLevelWidth);

	//SBI
	mSBI = mPyramid.getTopLevel();
	while(mSBI.cols > FLAGS_SBIMaxWidth)
	{
		cv::Mat1b temp;
		cv::pyrDown(mSBI,temp);
		mSBI = temp;
	}

	//SBI derivatives
	cvutils::CalculateDerivatives(mSBI, mSBIdx, mSBIdy);

	//Extract key points
	mKeypoints->resize(mPyramid.getOctaveCount());

	for(int octave=0; octave<mPyramid.getOctaveCount(); octave++)
	{
	    const int scale = 1<<octave;

	    //Create ROI
	    //cv::Rect roiRect(kNoFeatureBorderSize,kNoFeatureBorderSize, mPyramid[octave].cols-2*kNoFeatureBorderSize, mPyramid[octave].rows-2*kNoFeatureBorderSize);
	    //cv::Mat1b roi = mPyramid[octave](roiRect);

	    //FAST
	    std::vector<cv::KeyPoint> keypoints;
		//cv::FAST(roi, keypoints, kFASTThreshold, false);
	    cv::FAST(mPyramid[octave], keypoints, FLAGS_FASTThreshold, true);

	    //int maxX = mPyramid[octave].cols - PatchWarper::kPatchSize - 1;
	    //int maxY = mPyramid[octave].rows - PatchWarper::kPatchSize - 1;

		//Store features
		mKeypoints->at(octave).clear();
	    for(auto &keypoint : keypoints)
	    {
	    	//if(keypoint.pt.x < PatchWarper::kPatchSize || keypoint.pt.y < PatchWarper::kPatchSize || keypoint.pt.x > maxX || keypoint.pt.y > maxY)
	    	//	continue;

	    	Eigen::Vector2f pos((int)(scale*keypoint.pt.x), (int)(scale*keypoint.pt.y));

			mKeypoints->at(octave).emplace_back(pos, (int)keypoint.response, octave);
	    }
    }
}

std::unique_ptr<Keyframe> Keyframe::copyWithoutFeatures() const
{
	std::unique_ptr<Keyframe> newFrame(new Keyframe());

	newFrame->mTimestamp = mTimestamp;
	newFrame->mSBI = mSBI;
	newFrame->mSBIdx = mSBIdx;
	newFrame->mSBIdy = mSBIdy;
	newFrame->mColorImage = mColorImage;
	newFrame->mPyramid = mPyramid;
	newFrame->mKeypoints = mKeypoints;
	
	return newFrame;
}

void Keyframe::removeMeasurement(FeatureMeasurement *m)
{
	auto it = mMeasurements.begin(), end = mMeasurements.end();
	for (; it != end; ++it)
	{
		if (*it == m)
			break; //Found!
	}

	if (it != end)
	{
		mMeasurements.erase(it);
	}
	else
	{
		MYAPP_LOG << "Ahhh!!!! Attempted to remove a measurement that is not in the frame.\n";
	}
}


}
