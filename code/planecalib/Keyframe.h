/*
 * SlamKeyFrame.h
 *
 *  Created on: 8.2.2014
 *      Author: dan
 */

#ifndef KEYFRAME_H_
#define KEYFRAME_H_

#include <gflags/gflags.h>
#include <Eigen/Dense>
#include "eutils.h"
#include "ImagePyramid.h"

namespace planecalib {

class FeatureMeasurement;

///////////////////////////////////
// Classes
class Keyframe
{
public:
	typedef Eigen::Matrix<uchar, Eigen::Dynamic, 32, Eigen::RowMajor> EigenDescriptorMatrix;
	typedef Eigen::Map<EigenDescriptorMatrix> EigenDescriptorMap;

	Keyframe();
	Keyframe(const Keyframe &copyFrom);
	~Keyframe();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void init(const cv::Mat3b &imageColor, const cv::Mat1b &imageGray);
	std::unique_ptr<Keyframe> copyWithoutFeatures() const;

	double getTimestamp() const {return mTimestamp;}
	void setTimestamp(double value) {mTimestamp=value;}

	const cv::Mat1b &getSBI() const {return mSBI;}
	const cv::Mat1s &getSBIdx() const {return mSBIdx;}
	const cv::Mat1s &getSBIdy() const {return mSBIdy;}
	const cv::Mat3b &getColorImage() const {return mColorImage;}
	const ImagePyramid1b &getPyramid() const {return mPyramid;}
	const cv::Mat1b &getImage(int octave) const {return mPyramid[octave];}
	
	int getOctaveCount() const { return mPyramid.getOctaveCount(); }
	Eigen::Vector2i getImageSize() const { return Eigen::Vector2i(mPyramid[0].cols, mPyramid[0].rows); }

	//Note: mKeyPoints never changes and is therefore a shared_ptr between all the copies of this frame (just like the images)
	//		That way pointers to key points will remain valid for all copies of the frame.
	std::vector<cv::KeyPoint> &getKeypoints(int octave) const {return (*mKeypoints)[octave];}
	
	//Note: descriptors are shared between all copies of the frame
	const EigenDescriptorMap &getDescriptors(int octave) const
	{ 
		return mDescriptorsEigen[octave];
	}

	const Eigen::Matrix3fr &getPose() const {return mPose;}
	Eigen::Matrix3fr &getPose() { return mPose; }
	void setPose(const Eigen::Matrix3fr &pose) { mPose = pose; }

	std::vector<FeatureMeasurement *> &getMeasurements() {return mMeasurements;}
	const std::vector<FeatureMeasurement *> &getMeasurements() const {return mMeasurements;}

	void removeMeasurement(FeatureMeasurement *m);

	////////////////////////////////////
	// BA params
	Eigen::Matrix3dr mParamsPose;

protected:
	double mTimestamp;

	cv::Mat1b mSBI;
	cv::Mat1s mSBIdx;
	cv::Mat1s mSBIdy;

	cv::Mat3b mColorImage;

	ImagePyramid1b mPyramid;

	//Note: mKeyPoints never changes and is therefore a shared_ptr between all the copies of this frame (just like the images)
	//		That way pointers to key points will remain valid for all copies of the frame.
	std::shared_ptr<std::vector<std::vector<cv::KeyPoint>>> mKeypoints;

	std::vector<cv::Mat1b> mDescriptors;
	std::vector<EigenDescriptorMap> mDescriptorsEigen;

	Eigen::Matrix3fr mPose; //image = mPose * world

	std::vector<FeatureMeasurement *> mMeasurements;

public:
	Eigen::Matrix3fr mPose3DR;
	Eigen::Vector3f mPose3DT;
};

} 

#endif 
