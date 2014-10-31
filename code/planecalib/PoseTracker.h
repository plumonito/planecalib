
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

class PoseTracker
{
public:
	void init(const Eigen::Vector2i &imageSize, int octaveCount);
	void resetTracking(Map *map, const Eigen::Matrix3fr &initialPose);

	int getMatcherSearchRadius() const {return mMatcherSearchRadius;}

	bool trackFrame(std::unique_ptr<Keyframe> frame);

	void resync();

	bool isLost() const { return mIsLost; }

	const Eigen::Matrix3fr &getCurrentPose() const { return mCurrentPose; }
	void setCurrentPose(const Eigen::Matrix3fr &pose) { mCurrentPose = pose; }

	const Eigen::Vector2i getImageSize() const {return mImageSize;}
	const int getOctaveCount() const {return mOctaveCount;}

	const Keyframe *getFrame() const { return mFrame.get(); }

	//const cv::Matx23f &getFrameToLastSimilarity() const { return mSimilarityInv; }
	//const cv::Matx23f &getLastToFrameSimilarity() const { return mSimilarity; }

	const FeatureMatch *getMatch(const Feature *feature) const;
	const std::vector<FeatureMatch> &getMatches() const { return mMatches; }

	int getMatchInlierCount() const { return mMatchInlierCount; }
	const std::vector<MatchReprojectionErrors> &getReprojectionErrors() const { return mReprojectionErrors; }

protected:
	/////////////////////////////////////////////////////
	// Protected members

	bool mIsLost;

	Map *mMap;

	Eigen::Vector2i mImageSize;
	int mOctaveCount;

	int mMatcherSearchRadius; //Contrary to the flag, this is in image pixel units

	Eigen::Matrix3fr mCurrentPose;

	//Data from the previous frame
	//Only inliers are kept here
	//std::unique_ptr<FrameTrackingData> mLastTrackedFrameDat;
	std::unique_ptr<Keyframe> mLastFrame;
	std::vector<FeatureMatch> mLastMatches;

	//Data from the current frame
	std::unique_ptr<Keyframe> mFrame;

	std::vector<std::vector<FeatureProjectionInfo>> mFeaturesInView; //Outer vector is of octaves, inner of projections

	std::vector<FeatureMatch> mMatches;

	int mMatchInlierCount;
	std::vector<MatchReprojectionErrors> mReprojectionErrors;	//Same index as mMatches

	/////////////////////////////////////////////////////
	// Protected methods

	typedef std::vector<FeatureProjectionInfo *> TSortedFeaturesCell;
	void sortFeaturesInOctave(const std::vector<FeatureProjectionInfo> &previousMatches, 
			const std::vector<FeatureProjectionInfo> &featuresInOctave,
			std::vector<TSortedFeaturesCell> &featureGrid);
	void matchFeaturesInOctave(int maxFeatureCount,
			std::vector<TSortedFeaturesCell> &featureGrid);

	void updateFeatureProjections(const Eigen::Matrix3fr &pose,
			std::vector<FeatureProjectionInfo> &featureProjections);
	void findMatches(const int octave, const std::vector<std::pair<Feature *, cv::KeyPoint*>> &matches);
};

} /* namespace dtslam */

#endif /* POSETRACKER_H_ */
