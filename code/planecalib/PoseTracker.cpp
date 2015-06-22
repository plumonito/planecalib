/*
 * PoseTracker.cpp
 *
 *  Created on: 9.2.2014
 *      Author: dan
 */

#include "PoseTracker.h"

#include <limits>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>
#include "Profiler.h"
#include "cvutils.h"
#include "log.h"
#include "HomographyEstimation.h"
#include "PnpEstimation.h"
#include "flags.h"

namespace planecalib {

PoseTracker::~PoseTracker()
{

}

void PoseTracker::init(const Eigen::Vector2i &imageSize, int octaveCount)
{
	mImageSize = imageSize;
	mOctaveCount = octaveCount;

	//mIsLost = false;

	//Translate search radius to normalized units
	mMatcherSearchRadius = (1<<(octaveCount-1)) * FLAGS_MatcherPixelSearchDistance;
	MYAPP_LOG << "PoseTracker: Matcher search radius = " << mMatcherSearchRadius << "px in image res\n";

	//mFeatureSortGridSize = cv::Size2i(5,5);

	//Matcher
	//mMatcher.reset(new FeatureMatcher());
	//mMatcher->setCamera(mCamera);

	//Model refiners
	mHomographyEstimator.reset(new HomographyEstimation());

	//mPoseEstimator.reset(new PoseEstimator());
	//mPoseEstimator->init(mCamera, (float)FLAGS_TrackerOutlierPixelThreshold);
}

const Eigen::Matrix3fr &PoseTracker::getCurrentPose2D() const 
{ 
	return mPose2D; 
}

void PoseTracker::resetTracking(Map *map, const Eigen::Matrix3fr &initialPose)
{
	mMap = map;
	mLastPose2D = mPose2D = initialPose;
	mIsLost = true;

	//Forget all previous matches
	mLastFrame.reset(NULL);
	mFrame.reset(NULL);
	mFeaturesInView.clear();
	mFeaturesInView.resize(mOctaveCount);
}

bool PoseTracker::trackFrame(double timestamp, const cv::Mat3b &imageColor, const cv::Mat1b &imageGray)
{
	ProfileSection s("trackFrame");

	//Save old data 
	mLastPose2D = mPose2D;
	mLastFrame.reset(NULL);
	if (mFrame)
	{
		mLastFrame = std::move(mFrame);

		//Remove outliers
		std::remove_if(mLastFrame->getMatches().begin(), mLastFrame->getMatches().end(), 
			[](FeatureMatch &match){ return !match.getReprojectionErrors().isInlier; });
	}

	//Reset new frame data
	mFrame.reset(new TrackingFrame());
	mFrame->initImageData(imageColor, imageGray);
	mFrame->setTimestamp(timestamp);

	mFeaturesInView.clear();
	mFeaturesInView.resize(mOctaveCount);

	//Optical alignment
	Eigen::Matrix3fr poseGuess = mLastPose2D;
	Eigen::Matrix3fr opticalHomography; //p_now = opticalHomography*p_last
	if (mLastFrame)
	{
		estimateSimilarityFromLastFrame(*mFrame, opticalHomography);
	}
	else
	{
		opticalHomography = Eigen::Matrix3fr::Identity();
	}

	//Matches
	findMatches(opticalHomography, poseGuess);

	//Estimate pose
	if (mMap->getIs3DValid())
		trackFrame3D();
	else
		trackFrameHomography(poseGuess);

	//Build match map
	mFrame->createMatchMap();

	return !mIsLost;
}

bool PoseTracker::estimateSimilarityFromLastFrame(const TrackingFrame &frame, Eigen::Matrix3fr &similarity_)
{
	cv::Matx23f similarity = cv::Matx23f::eye(); //Identity is used as the initial guess

	ProfileSection ss("estimateSimilarity");
	bool res = mHomographyEstimator->estimateSimilarityDirect(mLastFrame->getSBI(), mLastFrame->getSBIdx(), mLastFrame->getSBIdy(), frame.getSBI(), similarity);

	//Scale similarity matrix
	//similarity = scaleUp*similarity*scaleDown
	const float scale = (float)mImageSize[0] / frame.getSBI().cols;
	similarity(0, 2) *= scale;
	similarity(1, 2) *= scale;
	similarity_ << similarity(0, 0), similarity(0, 1), similarity(0, 2), similarity(1, 0), similarity(1, 1), similarity(1, 2), 0, 0, 1;
	return res;
}

void PoseTracker::findMatches(const Eigen::Matrix3fr &opticalHomography, const Eigen::Matrix3fr &poseGuess)
{
	const int kDistanceThresholdSq0 = 40 * 40;
	const float kRatioThreshold = 0.8f;
	const int kMinScoreThreshold = 50;

	mTotalMatchAttempts = mTotalMatchSuccess = 0;

	std::unordered_set<const Feature*> featuresToIgnore;

	//Get features in view
	mMap->getFeaturesInView(opticalHomography, mCamera, poseGuess, mOctaveCount, featuresToIgnore, mFeaturesInView);
	if (mFeaturesInView.empty() || mFeaturesInView[0].empty())
		return;

	//Warp
	Keyframe *refFrame = &mFeaturesInView[0][0].getSourceMeasurement()->getKeyframe();
	Eigen::Matrix3fr warpHomography = poseGuess * refFrame->getPose().inverse();

	mFrame->createKeypoints(opticalHomography, mCamera, warpHomography);

	Eigen::Matrix3fr warpHomographyInv = warpHomography.inverse();

	//Match
	//int octave = 0;
	for (int octave = mOctaveCount - 1; octave >= 0; octave--)
	{
		const int scale = 1 << octave;
		const int kDistanceThresholdSq = kDistanceThresholdSq0 * (scale*scale);
		{
			ProfileSection sm("matching");

			auto &imgKeypoints = mFrame->getWarpedKeypoints(octave);
			auto &imgDesc = mFrame->getWarpedDescriptors(octave);
			//std::vector<cv::Point2f> refPoints, imgPoints;

			for (auto &projection : mFeaturesInView[octave])
			{
				auto &feature = projection.getFeature();
				auto &m = *projection.getSourceMeasurement();

				auto &refDesc_i = m.getDescriptor();

				mTotalMatchAttempts++;

				//Find best match
				int bestScore = std::numeric_limits<int>::max();
				cv::KeyPoint bestPosition;
				const uchar *bestDescriptor;
				int secondScore = std::numeric_limits<int>::max();
				cv::Hamming distFunc;
				for (int j = 0; j < (int)imgKeypoints.size(); j++)
				{
					//auto diff = projection.getPosition() - eutils::FromCV(imgKeypoints[j].pt);
					auto diff = m.getPosition() - eutils::FromCV(imgKeypoints[j].pt);
					if (diff.squaredNorm() < kDistanceThresholdSq || mIsLost)
					{
						const uchar *imgDesc_j = &imgDesc(j, 0);
						int score = distFunc(refDesc_i.data(), imgDesc_j, 32);
						if (score < bestScore)
						{
							secondScore = bestScore;
							bestScore = score;
							bestPosition = imgKeypoints[j];
							bestDescriptor = imgDesc_j;
						}
						else if (score < secondScore)
						{
							secondScore = score;
						}
					}
				}

				//Is match good enough?
				bool add = false;
				//MYAPP_LOG << bestScore << "/" << secondScore << "\n";
				//if (bestScore < std::numeric_limits<int>::max())
				if (bestScore < kMinScoreThreshold)
				{
					if (bestScore < kRatioThreshold * secondScore)
						add = true;
				}

				if (add)
				{
					mTotalMatchSuccess++;

					//Unwarp position
					//Eigen::Vector2f realPosition = eutils::HomographyPoint(warpHomography, eutils::FromCV(bestPosition.pt));
					Eigen::Vector2f realPosition = mFrame->warpKey2Img(eutils::FromCV(bestPosition.pt));

					mFrame->getMatches().push_back(FeatureMatch(&m, octave, bestPosition, realPosition, 0));
				}
			}
		}
	}

	//MYAPP_LOG << "Matches=" << refPoints.size() << "\n";
}

bool PoseTracker::trackFrameHomography(const Eigen::Matrix3fr &poseGuess)
{
	const int kMinInlierCount = 20;
	ProfileSection s("trackFrameHomography");

	int matchCount = mFrame->getMatches().size();

	//Homography
	if (matchCount < kMinInlierCount)
	{
		MYAPP_LOG << "Lost in 2D, only " << matchCount << " matches\n";
		mIsLost = true;
	}
	else
	{
		//Create cv vectors
		std::vector<Eigen::Vector2f> refPoints, imgPoints;
		std::vector<float> scales;
		for (auto &match : mFrame->getMatches())
		{
			refPoints.push_back(match.getFeature().getPosition() - mCamera.getPrincipalPoint());
			imgPoints.push_back(match.getPosition() - mCamera.getPrincipalPoint());
			scales.push_back((float)(1<<match.getOctave()));
		}

		//Eigen::Matrix<uchar,Eigen::Dynamic,1> mask(refPoints.size());
		//cv::Mat1b mask_cv(refPoints.size(), 1, mask.data());

		Eigen::Matrix3fr H;
		if (mIsLost || mForceRansac)
		{
			ProfileSection s("homographyRansac");

			HomographyRansac ransac;
			ransac.setParams(3, 10, 100, (int)(0.99f * matchCount));
			ransac.setData(&refPoints, &imgPoints, &scales);
			ransac.doRansac();
			H = ransac.getBestModel().cast<float>();
			MYAPP_LOG << "Homography ransac: inliers=" << ransac.getBestInlierCount() << "/" << matchCount << "\n";
		}
		else
			H = poseGuess;

		//Refine
		HomographyEstimation hest;
		std::vector<bool> inliersVec;
		{
			ProfileSection s("refineHomography");
			H = hest.estimateCeres(H, imgPoints, refPoints, scales, 2.5, inliersVec);
		}
		//std::vector<FeatureMatch> goodMatches;
		//int inlierCountBefore = mask.sum();
		mMatchInlierCount = 0;
		for (int i = 0; i<(int)matchCount; i++)
		{
			if (inliersVec[i])
				mMatchInlierCount++;
				//goodMatches.push_back(mMatches[i]);
		}
		//inlierCountAfter = goodMatches.size();
		//mMatches = std::move(goodMatches);
		//MYAPP_LOG << "Inliers before=" << inlierCountBefore << ", inliers after=" << inlierCountAfter << "\n";

		if (mMatchInlierCount > kMinInlierCount)
		{
			mPose2D = H;
			mIsLost = false;
		}
		else
		{
			MYAPP_LOG << "Lost in 2D, only " << mMatchInlierCount << " inliers\n";
			mIsLost = true;
		}
	}

	//Eval
	for (auto &match : mFrame->getMatches())
	{
		match.getReprojectionErrors().isInlier = true;
	}

	return !mIsLost;
}

bool PoseTracker::trackFrame3D()
{
	const int kMinInlierCount = 20;
	ProfileSection s("trackFrame3D");

	int matchCount = mFrame->getMatches().size();

	if (matchCount < kMinInlierCount)
	{
		MYAPP_LOG << "Lost in 3D, only " << matchCount << " matches\n";
		mIsLost = true;
	}
	else
	{
		std::vector<Eigen::Vector3f> refPoints;
		std::vector<Eigen::Vector2f> imgPoints;
		std::vector<float> scales;

		for (auto &match : mFrame->getMatches())
		{
			refPoints.push_back(match.getFeature().mPosition3D);
			imgPoints.push_back(match.getPosition());
			scales.push_back((float)(1 << match.getOctave()));
		}

		if (mIsLost)
		{
			PnPRansac ransac;
			ransac.setParams(3, 10, 100, (int)(0.9f*matchCount));
			ransac.setData(&refPoints, &imgPoints, &scales, mMap->mCamera.get());
			ransac.doRansac();
			mPose3D.R = ransac.getBestModel().first.cast<float>();
			mPose3D.t = ransac.getBestModel().second.cast<float>();
		}

		PnPRefiner refiner;
		refiner.setCamera(mMap->mCamera.get());
		refiner.setOutlierThreshold(3);
		std::vector<MatchReprojectionErrors> reprojectionErrors;
		refiner.refinePose(mFrame->getMatches(), mPose3D.R, mPose3D.t, mMatchInlierCount, reprojectionErrors);

		//MYAPP_LOG << "Inliers before=" << inlierCountBefore << ", inliers after=" << inlierCountAfter << "\n";

		//Convert to homography
		//Ref data could be computed just once
		Keyframe &refFrame = *mMap->getKeyframes()[0];
		Eigen::Matrix3fr K;
		K << mMap->mCamera->getFocalLength()[0], 0, 0, 0, mMap->mCamera->getFocalLength()[1], 0, 0, 0, 1;

		Eigen::Matrix3fr refR = refFrame.mPose3DR;
		Eigen::Vector3f refT = refFrame.mPose3DT - refR*Eigen::Vector3f::UnitZ();
		Eigen::Matrix3fr worldToRefH = K * (refR + refT*Eigen::Vector3f::UnitZ().transpose());

		Eigen::Matrix3fr &imgR = mPose3D.R;
		Eigen::Vector3f imgT = mPose3D.t - imgR*Eigen::Vector3f::UnitZ();
		Eigen::Matrix3fr worldToImgH = K * (imgR + imgT*Eigen::Vector3f::UnitZ().transpose());

		mPose2D = worldToImgH*worldToRefH.inverse();

		

		if (mMatchInlierCount > kMinInlierCount)
		{
			mIsLost = false;
		}
		else
		{
			MYAPP_LOG << "Lost in 3D, only " << mMatchInlierCount << " inliers\n";
			mIsLost = true;
		}
	}

	return !mIsLost;
}

//bool PoseTracker::trackFrame(std::unique_ptr<SlamKeyFrame> frame_)
//{
//	ProfileSection s("trackFrame");
//
//	//Save old data 
//	mLastFrame.reset(NULL);
//	mLastMatches.clear();
//	if (mFrame)
//	{
//		mLastFrame = std::move(mFrame);
//
//		for (int i = 0, end = mMatches.size(); i != end; ++i)
//		{
//			if (mReprojectionErrors[i].isInlier)
//			{
//				mLastMatches.push_back(mMatches[i]);
//			}
//		}
//	}
//
//	//Reset new frame data
//	mFrame = std::move(frame_);
//	mFeaturesInView.clear();
//	mFeaturesInView.resize(mOctaveCount);
//	mMatchAttempts.clear();
//	mMatches.clear();
//	mMatchMap.clear();
//	mReprojectionErrors.clear();
//
//	///////////////////////////////////////////////////////////////////////////////////
//	//Estimate initial similarity
//	//Small blurry image
//	if (estimateSimilarityFromLastFrame(*mFrame, mSimilarity))
//	{
//		cv::invertAffineTransform(mSimilarity, mSimilarityInv);
//		
//		//Prepare matcher
//		mMatcher->setFrame(mFrame.get(), mSimilarityInv);
//	}
//	else
//	{
//		mSimilarity = mSimilarityInv = cv::Matx23f::eye();
//
//		//Prepare matcher
//		mMatcher->setFrame(mFrame.get());
//	}
//	mMatcher->setFramePose(getCurrentPose());
//
//	mMatcher->setSearchDistance(mMatcherSearchRadius);
//
//	///////////////////////////////////////////////////////////////////////////////////
//	//Select features to match
//
//	//Remember previous matches so we don't project them
//	std::unordered_set<SlamFeature*> featuresToIgnore;
//	if (mLastFrame)
//	{
//		for (auto &match : mLastMatches)
//		{
//			featuresToIgnore.insert(&match.projection.getFeature());
//		}
//	}
//
//	//Find features in view
//	mActiveRegion->getFeaturesInView(
//			getCurrentPose(),
//			mFrame->getCameraModel(),
//			getOctaveCount(),
//			true,
//			featuresToIgnore,
//			mFeaturesInView);
//	//Debug: Show count of the number of features
//	{
//		std::vector<int> featureInViewCount2D(getOctaveCount(),0);
//		std::vector<int> featureInViewCount3D(getOctaveCount(), 0);
//		for (int octave = getOctaveCount() - 1; octave >= 0; --octave)
//		{
//			for (auto &projection : mFeaturesInView[octave])
//			{
//				if (projection.getFeature().is3D())
//					featureInViewCount3D[octave]++;
//				else
//					featureInViewCount2D[octave]++;
//			}
//		}
//		DTSLAM_LOG << "3D Features in view by octave: " << featureInViewCount3D << ".\n";
//		DTSLAM_LOG << "2D Features in view by octave: " << featureInViewCount2D << ".\n";
//	}
//
//	//Add all matches from the previous frame
//	std::vector<std::vector<FeatureProjectionInfo>> previousMatches;
//	previousMatches.resize(mOctaveCount);
//	if(mLastFrame)
//	{
//		for (auto &match : mLastMatches)
//		{
//			auto &m = match.measurement;
//			
//			if (m.getFeature().getStatus() == SlamFeatureStatus::Invalid)
//				continue; //Ignore features that have moved to the garbage
//
//			previousMatches[m.getOctave()].push_back(FeatureProjectionInfo::CreatePreviousMatch(&m.getFeature(), const_cast<SlamFeatureMeasurement*>(match.sourceMeasurement), m.getOctave(), match.trackLength + 1, m.getPositions()));
//		}
//	}
//
//
//	///////////////////////////////////////////////////////////////////////////////////
//	//Do first pass (using the help of the similarity)
//	//For now we do only one pass. PTAM does two passes, one with the help of the similarity for low resolution features and one for high resolution features
//	{
//		//ProfileSection s("Pass1");
//
//		//Go through octaves
//		int featuresNeeded = FLAGS_TrackerMaxFeatures;
//
//		{
//			ProfileSection s("matching");
//
//			for(int octave=mOctaveCount-1; octave>=0; --octave)
//			{
//				int totalInOctaveCount = (int)(mFeaturesInView[octave].size() + previousMatches[octave].size());
//				int featuresNeededOctave = std::min(featuresNeeded, std::min(totalInOctaveCount, FLAGS_TrackerMaxFeaturesPerOctave));
//				std::vector<TSortedFeaturesCell> featureGrid;
//				std::vector<FeatureProjectionInfo*> features2D;
//
//				//Sort and match
//				sortFeaturesInOctave(previousMatches[octave], mFeaturesInView[octave], featureGrid, features2D);
//				matchFeaturesInOctave(featuresNeededOctave, featureGrid, features2D);
//
//				featuresNeeded -= featuresNeededOctave;
//				if(featuresNeeded <= 0)
//					break;
//			}
//		}
//
//		//Get matching results
//		mMatchAttempts = std::move(mMatcher->getMatchAttempts());
//		mMatches = std::move(mMatcher->getMatches());
//		
//		//Create the match map
//		for (auto &match : mMatches)
//		{
//			mMatchMap.insert(std::make_pair(&match.measurement.getFeature(), &match));
//		}
//
//		//Debug: show match count
//		int foundMatchCount=0, foundKeyPointCount=0;
//		std::vector<int> foundCountByOctave(mOctaveCount,0);
//
//		foundMatchCount = mMatches.size();
//		int count3D = 0;
//		int count2D = 0;
//		for (auto &match : mMatches)
//		{
//			if (match.measurement.getFeature().is3D())
//				count3D++;
//			else
//				count2D++;
//			foundCountByOctave[match.measurement.getOctave()]++;
//			foundKeyPointCount += match.measurement.getPositionCount();
//		}
//		float percentage = (foundMatchCount) ? (100*foundKeyPointCount / (float)foundMatchCount) : 0.0f;
//		DTSLAM_LOG << "Found " << (foundMatchCount) << " matches (" << count3D << " 3D + " << count2D << " 2D) with " << percentage << "% key points";
//		for(int octave=mOctaveCount-1; octave>=0; --octave)
//			DTSLAM_LOG << ", octave " << octave << "=" << foundCountByOctave[octave];
//		DTSLAM_LOG << "\n";
//
//		if (foundMatchCount < 5)
//		{
//			DTSLAM_LOG << "Not enough matches, aborting tracking.\n";
//			mPoseType = EPoseEstimationType::Invalid;
//			mMatchInlierCount = 0;
//			mEssentialReferenceFrame = NULL;
//			mReprojectionErrors.resize(mMatches.size());
//			mReprojectionErrorsForRotation.resize(mMatches.size());
//		}
//		else
//		{
//			//Estimate pose
//			mPoseEstimator->setPreviousPose(getCurrentPose());
//			mPoseEstimator->fitModels(mMatches);
//
//			if (mPoseEstimator->getPoseType() == EPoseEstimationType::Invalid)
//			{
//				DTSLAM_LOG << "Pose estimation failed, aborting tracking.\n";
//				mPoseType = EPoseEstimationType::Invalid;
//				mMatchInlierCount = 0;
//				mEssentialReferenceFrame = NULL;
//				mReprojectionErrors.resize(mMatches.size());
//				mReprojectionErrorsForRotation.resize(mMatches.size());
//			}
//			else
//			{
//				mPoseType = mPoseEstimator->getPoseType();
//				mCurrentPose.set(mPoseEstimator->getPose());
//				mEssentialReferenceFrame = mPoseEstimator->getEssentialReferenceFrame();
//
//				//Copy residuals
//				mMatchInlierCount = mPoseEstimator->getInlierCount();
//				mReprojectionErrors = std::move(mPoseEstimator->getReprojectionErrors());
//				mReprojectionErrorsForRotation = std::move(mPoseEstimator->getPureRotationReprojectionErrors());
//			}
//		}
//	}
//
//	mFrame->setPose(std::unique_ptr<Pose3D>(new FullPose3D(getCurrentPose())));
//
//	return mMatchInlierCount > 20;
//}
//
//bool PoseTracker::estimateSimilarityFromLastFrame(const SlamKeyFrame &frame, cv::Matx23f &similarity)
//{
//	similarity = cv::Matx23f::eye(); //Identity is used as the initial guess
//	if(mLastFrame)
//	{
//		ProfileSection ss("estimateSimilarity");
//		bool res = mHomographyEstimator->estimateSimilarityDirect(mLastFrame->getSBI(), mLastFrame->getSBIdx(), mLastFrame->getSBIdy(), frame.getSBI(), similarity);
//
//		//Scale similarity matrix
//		//similarity = scaleUp*similarity*scaleDown
//		const int scale = mImageSize.width / frame.getSBI().cols;
//		similarity(0,2) *= scale;
//		similarity(1,2) *= scale;
//		return res;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//void PoseTracker::sortFeaturesInOctave(const std::vector<FeatureProjectionInfo> &previousMatches, 
//		const std::vector<FeatureProjectionInfo> &featuresInOctave,
//		std::vector<TSortedFeaturesCell> &featureGrid,
//		std::vector<FeatureProjectionInfo*> &features2D)
//{
//	ProfileSection s("sortInOctave");
//	const int cellCount = mFeatureSortGridSize.area();
//
//	featureGrid.resize(cellCount);
//
//	const int cellWidth = mImageSize.width / mFeatureSortGridSize.width;
//	const int cellHeight = mImageSize.height / mFeatureSortGridSize.height;
//
//	//Randomize
//	std::vector<FeatureProjectionInfo*> randomProjections;
//	for(auto &projection : featuresInOctave)
//		randomProjections.push_back(const_cast<FeatureProjectionInfo*>(&projection));
//	std::random_shuffle(randomProjections.begin(), randomProjections.end());
//
//	//Separate into cells
//	for(auto &projectionPtr : randomProjections)
//	{
//		auto &projection = *projectionPtr;
//
//		if(projection.getType() == EProjectionType::EpipolarLine)
//		{
//			//Epipolar line
//			features2D.push_back(&projection);
//		}
//		else
//		{
//			//Point projection
//			//Add first projected positions
//			const auto pos = projection.getPointData().positions[0];
//
//			int celli = (int)(pos.x / cellWidth);
//			if (celli < 0 || celli >= mFeatureSortGridSize.width)
//				continue;
//
//			int cellj = (int)(pos.y / cellHeight);
//			if (cellj < 0 || cellj >= mFeatureSortGridSize.height)
//				continue;
//
//			int cellIdx = cellj*mFeatureSortGridSize.width + celli;
//
//			featureGrid[cellIdx].push_back(&projection);
//		}
//	}
//
//	//Add previous matches last
//	for (auto &projection : previousMatches)
//	{
//		//Point projection
//		//Add first projected positions
//		const auto pos = projection.getPointData().positions[0];
//
//		int celli = (int)(pos.x / cellWidth);
//		if (celli < 0 || celli >= mFeatureSortGridSize.width)
//			continue;
//
//		int cellj = (int)(pos.y / cellHeight);
//		if (cellj < 0 || cellj >= mFeatureSortGridSize.height)
//			continue;
//
//		int cellIdx = cellj*mFeatureSortGridSize.width + celli;
//
//		featureGrid[cellIdx].push_back(const_cast<FeatureProjectionInfo*>(&projection));
//	}
//}
//
//void PoseTracker::matchFeaturesInOctave(int maxFeatureCount,
//		std::vector<TSortedFeaturesCell> &featureGrid,
//		std::vector<FeatureProjectionInfo *> &features2D)
//{
//	ProfileSection s("matchFeaturesInOctave");
//	const int cellWidth = mImageSize.width / mFeatureSortGridSize.width;
//	const int cellHeight = mImageSize.height / mFeatureSortGridSize.height;
//
//	int featuresNeeded = maxFeatureCount;
//
//	//Keep track of match counts
//	std::vector<FeatureProjectionInfo*> attempts;
//	attempts.reserve(featuresNeeded);
//
//	int totalSearchPoint = 0;
//	int totalSearchEpipolar = 0;
//	int totalFoundPoint = 0;
//	int totalFoundEpipolar = 0;
//
//	//Match point projections
//	{
//		ProfileSection ss("selectAttempts");
//
//		//Valid cells ( = cells with features)
//		std::list<TSortedFeaturesCell*> validCells; 
//		for (auto &cell : featureGrid)
//		{
//			if (!cell.empty())
//				validCells.push_back(&cell);
//		}
//
//		while (featuresNeeded>0 && !validCells.empty())
//		{
//			int validCellCount = validCells.size();
//			int attemptsPerCell = (featuresNeeded / validCellCount)+1;
//
//			for (auto validIt = validCells.begin(), validEnd=validCells.end(); validIt != validEnd;)
//			{
//				auto &cell = **validIt;
//
//				//Match
//				int attemptCount = std::min((int)cell.size(), attemptsPerCell);
//				for (int i = 0; i != attemptCount; ++i)
//				{
//					attempts.push_back(cell.back());
//					cell.pop_back();
//				}
//				totalSearchPoint += attemptCount;
//				featuresNeeded -= attemptCount;
//
//				//Exit if we've matched enough
//				if (featuresNeeded < 1)
//					break;
//
//				//Advance cell (delete if empty)
//				if (cell.empty())
//				{
//					validIt = validCells.erase(validIt);
//					validEnd = validCells.end();
//				}
//				else
//					++validIt;
//			}
//		}
//
//		//Match epipolar projections
//
//		//This avoids any 2D matches when PoseUse2D is not enabled
//		if (FLAGS_PoseUse2D || mActiveRegion->getFeatures3D().size() < 50)
//		{
//			//Match epipolar features
//			while (featuresNeeded > 0 && !features2D.empty())
//			{
//				attempts.push_back(features2D.back());
//				features2D.pop_back();
//
//				totalSearchEpipolar++;
//				featuresNeeded--;
//			}
//		}
//	}
//
//	//Match
//	{
//		ProfileSection ss("matching");
//		for (auto projectionPtr : attempts)
//		{
//			auto &projection = *projectionPtr;
//
//			FeatureMatch *match;
//			match = mMatcher->findMatch(projection);
//			if (match)
//			{
//				//Matched!
//				if (projection.getType() == EProjectionType::EpipolarLine)
//					totalFoundEpipolar++;
//				else
//					totalFoundPoint++;
//			}
//		}
//	}
//
//	//DTSLAM_LOG << "Searching for " << (totalSearchPoint + totalSearchEpipolar) << " features (" << totalSearchPoint << " points + " << totalSearchEpipolar << " epipolar).\n";
//	//DTSLAM_LOG << "Found " << (totalFoundPoint + totalFoundEpipolar) << " matches (" << totalFoundPoint << " points + " << totalFoundEpipolar << " epipolar).\n";
//}
//
//void PoseTracker::updateFeatureProjections(const Pose3D &pose,
//		std::vector<FeatureProjectionInfo> &featureProjections)
//{
//	assert(false);
//	for(int i=0; i<(int)featureProjections.size(); ++i)
//	{
//		FeatureProjectionInfo &projection = featureProjections[i];
//		SlamFeature &feature = projection.getFeature();
//
//		if(feature.is3D())
//		{
//			//Update projection of 3D feature
//			//const cv::Point3f xc = pose.apply(feature.getPosition());
//			//const cv::Point2f uv = mCamera->projectFromWorld(xc);
//
//			//projection.setPosition(uv);
//			//TODO Update warp for 3D feature
//		}
//		else
//		{
//			//TODO:Update projection of 2D feature
//			//projection.position = SlamMap::Project2d(feature.getPosition(), pose.getRotation(), *mCamera);
//		}
//	}
//}

void PoseTracker::resync()
{
//	if(!mFrame || mPoseType == EPoseEstimationType::Invalid)
//		return;
//
//	//Refit position
//	mPoseEstimator->fitModels(mMatches);
//	
//	mCurrentPose.set(mPoseEstimator->getPose());
//	mFrame->setPose(std::unique_ptr<Pose3D>(new FullPose3D(getCurrentPose())));
//
//	//Copy residuals
//	mMatchInlierCount = mPoseEstimator->getInlierCount();
//	mReprojectionErrors = std::move(mPoseEstimator->getReprojectionErrors());
//	mReprojectionErrorsForRotation = std::move(mPoseEstimator->getPureRotationReprojectionErrors());
}

} /* namespace dtslam */
