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
#include "FeatureIndexer.h"

namespace planecalib {

const Feature &FeatureMatch::getFeature() const 
{
	return mSourceMeasurement->getFeature(); 
}

///////////////////////////////////
// TrackingFrame class
TrackingFrame::TrackingFrame()
{
}

TrackingFrame::~TrackingFrame()
{
}

void TrackingFrame::initImageData(const cv::Mat3b &imageColor, const cv::Mat1b &imageGray)
{
	mColorImage = imageColor;

	mOriginalPyramid.create(imageGray, FLAGS_PyramidMaxTopLevelWidth);

	//SBI
	mSBI = mOriginalPyramid.getTopLevel();
	while (mSBI.cols > FLAGS_SBIMaxWidth)
	{
		cv::Mat1b temp;
		cv::pyrDown(mSBI, temp);
		mSBI = temp;
	}

	//SBI derivatives
	cvutils::CalculateDerivatives(mSBI, mSBIdx, mSBIdy);
}

void TrackingFrame::createKeypoints(const Eigen::Matrix3fr &warpHomography)
{
	cv::Mat1b warpedImage;

	mWarpHomography = warpHomography;

	warpedImage.create(mOriginalPyramid[0].size());
	cv::warpPerspective(mOriginalPyramid[0], warpedImage, eutils::ToCV(warpHomography), mOriginalPyramid[0].size(), cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, 0);

	mWarpedPyramid.create(warpedImage, FLAGS_PyramidMaxTopLevelWidth);

	Keyframe::CreateKeypoints(mWarpedPyramid, mWarpedKeypoints, mWarpedDescriptorBuffers, mWarpedDescriptors);
}

void TrackingFrame::createMatchMap()
{
	mMatchMap.clear();
	for (auto &match : mMatches)
		mMatchMap.insert(std::make_pair(&match.getFeature(), &match));
}

///////////////////////////////////
// Keyframe class

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

	createKeypoints();
}

void Keyframe::init(const TrackingFrame &other)
{
	mColorImage = other.getColorImage();
	mPyramid = other.getOriginalPyramid();
	mSBI = other.getSBI();
	mSBIdx = other.getSBIdx();
	mSBIdy = other.getSBIdy();
	mTimestamp = other.getTimestamp();

	createKeypoints();
}

void Keyframe::CreateKeypoints(const ImagePyramid1b &pyramid, std::vector<std::vector<cv::KeyPoint>> &keypoints, std::vector<cv::Mat1b> &descriptorBuffers, std::vector<EigenDescriptorMap> &descriptors)
{
	keypoints.clear();
	descriptorBuffers.clear();
	descriptors.clear();

	//Extract key points
	keypoints.resize(pyramid.getOctaveCount());
	descriptorBuffers.resize(pyramid.getOctaveCount());
	descriptors.reserve(pyramid.getOctaveCount());

	cv::Ptr<cv::ORB> orb = cv::ORB::create(2000, 2, 1);
	orb->setEdgeThreshold(0);

	for (int octave = 0; octave < pyramid.getOctaveCount(); octave++)
	{
		const int scale = 1 << octave;

		//std::vector<cv::KeyPoint> keypointsAll;
		//cv::Mat1b descriptorsAll;
		std::vector<cv::KeyPoint> &keypointsAll = keypoints[octave];
		cv::Mat1b &descriptorsAll = descriptorBuffers[octave];

		//ORB features
		orb->detectAndCompute(pyramid[octave], cv::noArray(), keypointsAll, descriptorsAll);

		//Fix scale features
		for (int i = 0; i < (int)keypointsAll.size(); i++)
		{
			auto &keypoint = keypointsAll[i];
			void *descriptor = &descriptorsAll(i, 0);

			keypoint.octave = octave;
			keypoint.pt = scale*keypoint.pt;
			keypoint.size *= scale;
		}

		if (descriptorBuffers[octave].empty())
			descriptorBuffers[octave].create(1, 32);

		//Eigen
		descriptors.push_back(EigenDescriptorMap(descriptorBuffers[octave].data, descriptorBuffers[octave].rows, descriptorBuffers[octave].cols));
	}
}

void Keyframe::createKeypoints()
{
	CreateKeypoints(mPyramid, *mKeypoints, mDescriptors, mDescriptorsEigen);
}

void Keyframe::freeSpace()
{
	mPyramid.release();
	mColorImage.release();
	mSBI.release();
	mSBIdx.release();
	mSBIdy.release();
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
