
#ifndef POSETRACKER_H_
#define POSETRACKER_H_

#include <memory>
#include <vector>
#include <list>
#include <unordered_map>

#include "Map.h"
#include "FeatureMatcher.h"
#include "PoseEstimationCommon.h"


namespace planecalib {

///////////////////////////////////
// Classes
class Keyframe;
class HomographyEstimation;

class PoseTracker
{
public:
	~PoseTracker();

	void init(const Eigen::Vector2i &imageSize, int octaveCount);
	void resetTracking(Map *map, const Eigen::Matrix3fr &initialPose);

	int getMatcherSearchRadius() const {return mMatcherSearchRadius;}

	bool trackFrame(double timestamp, const cv::Mat3b &imageColor, const cv::Mat1b &imageGray);

	void resync();

	bool isLost() const { return mIsLost; }

	const CameraModel &getCamera() const { return mCamera; }
	void setCamera(const CameraModel &value) { mCamera=value; }

	const Eigen::Matrix3fr &getCurrentPose2D() const;

	const Eigen::Vector2i getImageSize() const {return mImageSize;}
	const int getOctaveCount() const {return mOctaveCount;}

	const TrackingFrame *getFrame() const { return mFrame.get(); }

	//const cv::Matx23f &getFrameToLastSimilarity() const { return mSimilarityInv; }
	//const cv::Matx23f &getLastToFrameSimilarity() const { return mSimilarity; }

	int getMatchInlierCount() const { return mMatchInlierCount; }
	const std::vector<int> &getMatchInlierCountByOctave() const { return mMatchInlierCountByOctave; }

	bool mForceRansac;
	
	//stats
	int mTotalMatchAttempts;
	int mTotalMatchSuccess;

	//Pose
	const Eigen::Matrix3fr &getPose2D() const { return mPose2D; }
	const Pose3D &getPose3D() const { return mPose3D; }

protected:
	/////////////////////////////////////////////////////
	// Protected members

	bool mIsLost;

	CameraModel mCamera;
	Map *mMap;

	Eigen::Vector2i mImageSize;
	int mOctaveCount;

	int mMatcherSearchRadius; //Contrary to the flag, this is in image pixel units

	Eigen::Matrix3fr mPose2D; //This is a homography, used before calibration
	Pose3D mPose3D;	//This is a rigid pose, used after calibration

	Eigen::Matrix3fr mLastPose2D;

	std::unique_ptr<HomographyEstimation> mHomographyEstimator;

	//Data from the previous frame
	//Only inliers are kept here
	//std::unique_ptr<FrameTrackingData> mLastTrackedFrameDat;
	std::unique_ptr<TrackingFrame> mLastFrame;

	//Data from the current frame
	std::unique_ptr<TrackingFrame> mFrame;

	std::vector<std::vector<FeatureProjectionInfo>> mFeaturesInView; //Outer vector is of octaves, inner of projections

	int mMatchInlierCount;
	std::vector<int> mMatchInlierCountByOctave;

	/////////////////////////////////////////////////////
	// Protected methods

	bool estimateSimilarityFromLastFrame(const TrackingFrame &frame, Eigen::Matrix3fr &similarity);
	void findMatches(const Eigen::Matrix3fr &opticalHomography, const Eigen::Matrix3fr &poseGuess);
	bool trackFrameHomography(const Eigen::Matrix3fr &opticalHomography, const Eigen::Matrix3fr &poseGuess);
	bool trackFrame3D(const Eigen::Matrix3fr &opticalHomography, const Eigen::Matrix3fr &poseGuess);
};

} /* namespace dtslam */

#endif /* POSETRACKER_H_ */
