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
#include <unordered_map>
#include "eutils.h"
#include "ImagePyramid.h"
#include "PoseEstimationCommon.h"
#include "CameraModel.h"

namespace planecalib {

class Feature;
class FeatureMeasurement;

typedef Eigen::Matrix<uchar, Eigen::Dynamic, 32, Eigen::RowMajor> EigenDescriptorMatrix;
typedef Eigen::Map<EigenDescriptorMatrix> EigenDescriptorMap;

///////////////////////////////////
// FeatureMatch class
class FeatureMatch
{
public:
	FeatureMatch()
	{
	}

	FeatureMatch(const FeatureMeasurement *source, int octave_, const cv::KeyPoint &keypoint_, const Eigen::Vector2f &position_, int tracklength_) :
		mSourceMeasurement(source), mOctave(octave_), mKeypoint(keypoint_), mPosition(position_), mTrackLength(tracklength_)
	{}

	const FeatureMeasurement *getSourceMeasurement() const { return mSourceMeasurement; }
	const Feature &getFeature() const; 
	int getOctave() const { return mOctave; }
	const Eigen::Vector2f &getPosition() const { return mPosition; }
	const cv::KeyPoint &getKeypoint() const { return mKeypoint; }
	int getTrackLength() const { return mTrackLength; }
	MatchReprojectionErrors &getReprojectionErrors() { return mReprojectionErrors; }

protected:
	const FeatureMeasurement *mSourceMeasurement;
	int mOctave;
	cv::KeyPoint mKeypoint;
	Eigen::Vector2f mPosition;
	int mTrackLength;
	MatchReprojectionErrors mReprojectionErrors;
};

///////////////////////////////////
// Pose3D class
class Pose3D
{
public:
	Eigen::Matrix3fr R;
	Eigen::Vector3f t;
};

///////////////////////////////////
// TrackingFrame class
class TrackingFrame
{
public:
	TrackingFrame();
	~TrackingFrame();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void initImageData(const cv::Mat3b &imageColor, const cv::Mat1b &imageGray);

	//lastx = warpPose*keyx
	void createKeypoints(const Eigen::Matrix3fr &warpOpticalHomography, const CameraModel &camera, const Eigen::Matrix3fr &warpPose);

	const cv::Mat3b &getColorImage() const { return mColorImage; }
	const ImagePyramid1b &getOriginalPyramid() const { return mOriginalPyramid; }

	double getTimestamp() const { return mTimestamp; }
	void setTimestamp(double value) { mTimestamp = value; }

	const cv::Mat1b &getSBI() const { return mSBI; }
	const cv::Mat1s &getSBIdx() const { return mSBIdx; }
	const cv::Mat1s &getSBIdy() const { return mSBIdy; }

	const Eigen::Matrix3fr &getWarpHomography() const { return mWarpOpticalHomography; }
	const CameraModel &getWarpCamera() const { return mWarpCamera; }
	const Eigen::Matrix3fr &getWarpPose() const { return mWarpPose; }

	Eigen::Vector2f warpKey2Img(const Eigen::Vector2f &keyp);

	const ImagePyramid1b &getWarpedPyramid() const { return mWarpedPyramid; }

	const std::vector<cv::KeyPoint> &getWarpedKeypoints(int octave) const { return mWarpedKeypoints[octave]; }
	const EigenDescriptorMap &getWarpedDescriptors(int octave) const	{ return mWarpedDescriptors[octave]; }

	std::vector<FeatureMatch> &getMatches() { return mMatches; }
	const std::vector<FeatureMatch> &getMatches() const { return mMatches; }

	//Match map
	void createMatchMap();
	const std::unordered_map<const Feature*, FeatureMatch*> &getMatchMap() const { return mMatchMap; }
	const FeatureMatch *getMatch(const Feature *feature) const
	{
		auto it = mMatchMap.find(feature);
		return (it == mMatchMap.end()) ? NULL : it->second;
	}

protected:
	double mTimestamp;

	cv::Mat3b mColorImage;
	ImagePyramid1b mOriginalPyramid;

	cv::Mat1b mSBI;
	cv::Mat1s mSBIdx;
	cv::Mat1s mSBIdy;

	Eigen::Matrix3fr mWarpOpticalHomography;
	CameraModel mWarpCamera;
	Eigen::Matrix3fr mWarpPose;
	Eigen::Matrix3fr mWarpHomography;

	ImagePyramid1b mWarpedPyramid;

	std::vector<std::vector<cv::KeyPoint>> mWarpedKeypoints;

	std::vector<cv::Mat1b> mWarpedDescriptorBuffers;
	std::vector<EigenDescriptorMap> mWarpedDescriptors;

	std::vector<FeatureMatch> mMatches; //Position of these matches are given in original image coordinates.
	std::unordered_map<const Feature*, FeatureMatch *> mMatchMap; //Second is pointer into mMatches
};

///////////////////////////////////
// Keyframe class
class Keyframe
{
public:
	Keyframe();
	Keyframe(const Keyframe &copyFrom);
	~Keyframe();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void init(const cv::Mat3b &imageColor, const cv::Mat1b &imageGray);
	void init(const TrackingFrame &other);
	
	static void CreateKeypoints(const ImagePyramid1b &pyramid, std::vector<std::vector<cv::KeyPoint>> &keypoints, std::vector<cv::Mat1b> &descriptorBuffers, std::vector<EigenDescriptorMap> &descriptors);

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

	void freeSpace();

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

	void createKeypoints();

public:
	Eigen::Matrix3fr mPose3DR;
	Eigen::Vector3f mPose3DT;

	Eigen::Matrix3fr mGroundTruthPose3DR;
	Eigen::Vector3f mGroundTruthPose3DT;
};

} 

#endif 
