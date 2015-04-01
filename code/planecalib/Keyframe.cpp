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
		mKeypoints(new std::vector<std::vector<cv::KeyPoint>>())
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
	mDescriptors.resize(mPyramid.getOctaveCount());
	mDescriptorsEigen.reserve(mPyramid.getOctaveCount());

	for(int octave=0; octave<mPyramid.getOctaveCount(); octave++)
	{
	    const int scale = 1<<octave;
		std::vector<cv::KeyPoint> &keypoints = mKeypoints->at(octave);

	    //ORB features
		cv::ORB orb(500, 2, 1);
		orb(mPyramid[octave], cv::noArray(), keypoints, mDescriptors[octave]);

		//Fix scale features
		for(auto &keypoint : keypoints)
	    {
			keypoint.octave = octave;
			keypoint.pt = scale*keypoint.pt;
			keypoint.size *= scale;
	    }

		if (mDescriptors[octave].empty())
			mDescriptors[octave].create(1, 32);

		//Eigen
		mDescriptorsEigen.push_back(EigenDescriptorMap(mDescriptors[octave].data, mDescriptors[octave].rows, mDescriptors[octave].cols));
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

//cv::Matx<uchar, 1, 32> Keyframe::getDescriptor(int octave, int idx)
//{
//	assert(octave < (int)mDescriptors.size());
//	auto &desc = mDescriptors[octave];
//
//	assert(idx < desc.rows);
//	assert(32 == desc.cols);
//	uchar *row = desc.ptr(idx);
//
//	cv::Matx<uchar, 1, 32> res;
//	for (int i = 0; i < 32; i++)
//	{
//		res(0,i) = row[i];
//	}
//	
//	return res;
//}

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
