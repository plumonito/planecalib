
#include "PlaneCalibSystem.h"
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <ceres/rotation.h>

#include "Keyframe.h"
#include "Profiler.h"
#include "PoseTracker.h"
#include "HomographyCalibration.h"
#include "HomographyEstimation.h"
#include "BundleAdjuster.h"
#include "CalibratedBundleAdjuster.h"
//#include "CeresUtils.h"
//#include "FeatureGridIndexer.h"

#include "flags.h"

namespace planecalib
{

PlaneCalibSystem::~PlaneCalibSystem()
{

}
	
bool PlaneCalibSystem::init(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray)
{
	//Wait for other threads
	mExpanderCheckPending = false;
	if (mExpanderFuture.valid())
		mExpanderFuture.get();

	//Create first key frame
    std::unique_ptr<Keyframe> keyframe(new Keyframe());
    keyframe->init(imgColor, imgGray);
    keyframe->setTimestamp(timestamp);
    keyframe->setPose(Eigen::Matrix3fr::Identity());

	Eigen::Vector2i imageSize = keyframe->getImageSize();

    MYAPP_LOG << "Pyramid sizes: ";
	MYAPP_LOG << eutils::ToSize(imageSize);
    for(int octave=1; octave<keyframe->getPyramid().getOctaveCount(); octave++)
		MYAPP_LOG << ", " << keyframe->getImage(octave).size();
	MYAPP_LOG << "\n";

	MYAPP_LOG << "SBI size: " << keyframe->getSBI().size() << "\n";

	//Reset map
	mMap.reset(new Map());

	//Reset tracker
	mTracker.reset(new PoseTracker());
	mTracker->init(imageSize, keyframe->getOctaveCount());

	//Prepare map expander
	//mMapExpander.reset(new SlamMapExpander());
	//mMapExpander->init(camera, this);
	//mExpanderCheckPending = false;

	mCalib.reset(new HomographyCalibration());

	//Start map
	//Add keyframe
	Keyframe *pkeyframe = keyframe.get();
	mMap->addKeyframe(std::move(keyframe));

	Eigen::Matrix3fr poseInv = pkeyframe->getPose().inverse();

	//Create 2D features
	for (int octave = 0, end = pkeyframe->getPyramid().getOctaveCount(); octave != end; ++octave)
	{
		const int scale = 1 << octave;

		//cv::Size2i tileSize(scale*FLAGS_FrameKeypointGridSize, scale*FLAGS_FrameKeypointGridSize);
		//auto keypoints = FeatureGridIndexer<KeypointData>::ApplyNonMaximaSuppresion(pkeyframe->getKeypoints(octave), imageSize, tileSize, scale*16);
		auto keypoints = pkeyframe->getKeypoints(octave);
		for (uint i = 0; i < keypoints.size(); i++)
		{
			auto &kp = keypoints[i];
			mMap->createFeature(*pkeyframe, poseInv, Eigen::Vector2f(kp.pt.x, kp.pt.y), octave, &pkeyframe->getDescriptors(octave)(i,0));
		}

		MYAPP_LOG << "Created " << keypoints.size() << " features in octave " << octave << "\n";
	}

	mTracker->resetTracking(mMap.get(), mMap->getKeyframes().back()->getPose());

	return true;
}

//bool SlamSystem::init(const CameraModel *camera, std::unique_ptr<SlamMap> map)
//{
//	//Wait for other threads
//	if (mBAFuture.valid())
//		mBAFuture.get();
//	if (mExpanderFuture.valid())
//		mExpanderFuture.get();
//
//	//Reset map
//	mMap = std::move(map);
//	mPoseLog.clear();
//
//	//Get a reference keyframe
//	if (mMap->getRegions().empty())
//		throw std::runtime_error("Cannot init system with a map without regions.");
//	mActiveRegion = mMap->getRegions().front().get();
//	if (mActiveRegion->getKeyFrames().empty())
//		throw std::runtime_error("Cannot init system with a map without keyframes.");
//	
//	SlamKeyFrame *keyFrame = mMap->getRegions().front()->getKeyFrames().front().get();
//
//	//Reset tracker
//	mTracker.reset(new PoseTracker());
//	mTracker->init(camera, keyFrame->getImage(0).size(), keyFrame->getPyramid().getOctaveCount());
//	mTracker->setActiveRegion(mActiveRegion);
//	//mTracker->resetTracking(mActiveRegion->getKeyFrames().back()->getPose());
//	mTracker->resetTracking(mActiveRegion->getKeyFrames().front()->getPose());
//
//	//Prepare map expander
//	mMapExpander.reset(new SlamMapExpander());
//	mMapExpander->init(camera, this);
//	mMapExpander->setRegion(mActiveRegion);
//
//	return true;
//}
//


void PlaneCalibSystem::processImage(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray)
{
	//Reset profiler for each image
	//Profiler::Instance().reset();

	//Build keyframe structure
	std::unique_ptr<Keyframe> frame(new Keyframe());
	{
		ProfileSection ss("buildKeyFrame");
		frame->init(imgColor, imgGray);
		frame->setTimestamp(timestamp);
	}

	//Tracking
	bool trackingSuccesful=false;
	{
		//Read lock while tracking
		shared_lock<shared_mutex> lockRead(mMap->getMutex());

		//Track
		trackingSuccesful = mTracker->trackFrame(std::move(frame));
	}

	if (trackingSuccesful)
	{
		/* 
		// This replaces the feature-based homography with a direct estimation
		HomographyEstimation hest;
		cv::Matx33f t = eutils::ToCV(mTracker->getCurrentPose());
		//hest.estimateHomographyDirect(mMap->getKeyframes()[0]->getImage(0), frame->getImage(0), t);

		cv::Mat1f mm(3, 3, t.val);
		cv::findTransformECC(mMap->getKeyframes()[0]->getImage(0), mTracker->getFrame()->getImage(0), mm, cv::MOTION_HOMOGRAPHY, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 0.001));

		//mTracker->getFrame()->setPose(eutils::FromCV(t));
		mTracker->setCurrentPose(eutils::FromCV(t));
		*/

		mExpanderCheckPending = true;

		//Check distance
		const float kThreshold = 50 * 50;
		std::vector<Eigen::Vector2f> cornersOriginal;
		cornersOriginal.push_back(Eigen::Vector2f(0, 0));
		cornersOriginal.push_back(Eigen::Vector2f(0, mTracker->getImageSize().y()));
		cornersOriginal.push_back(Eigen::Vector2f(mTracker->getImageSize().x(), 0));
		cornersOriginal.push_back(Eigen::Vector2f(mTracker->getImageSize().x(), mTracker->getImageSize().y()));

		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> corners;
		Eigen::Matrix3fr poseInv = mTracker->getCurrentPose().inverse();
		for (auto &p : cornersOriginal)
			corners.emplace_back(p, eutils::HomographyPoint(poseInv, p));

		bool add = true;
		for (auto &frame_ : mMap->getKeyframes())
		{
			auto &frame = *frame_;
			float distSq = 0;
			for (auto &p : corners)
			{
				auto pp = eutils::HomographyPoint(frame.getPose(), p.second);
				distSq += (p.first - pp).squaredNorm();
			}

			if (distSq < kThreshold)
			{
				add = false;
				break;
			}
		}

		if (add)
		{
			createKeyframe();

			std::vector<Eigen::Matrix3fr> allPoses;
			for (auto &frame : mMap->getKeyframes())
			{
				allPoses.push_back(frame->getPose());
			}
			//mAllH.push_back(mTracker->getCurrentPose());

			//Calibrate
			mCalib->calibrate(allPoses, mTracker->getImageSize());
			//MYAPP_LOG << "K=" << K << "\n";

			// THIS DOESN'T WORK YET!!!
			//CalibratedBundleAdjuster ba;
			//ba.setMap(mMap.get());
			//ba.setOutlierThreshold(2.5f);
			//ba.setUseLocks(false);
			//for (auto &framePtr : mMap->getKeyframes())
			//{
			//	ba.addFrameToAdjust(*framePtr);
			//}
			//ba.setK(mCalib->getK().cast<double>().eval());
			//ba.bundleAdjust();
		}
	}
}

void PlaneCalibSystem::createKeyframe()
{
	auto frame_ = std::make_unique<Keyframe>(*mTracker->getFrame());
	Keyframe *frame = frame_.get();
	frame->setPose(mTracker->getCurrentPose());

	//Add keyframe to map
	mMap->addKeyframe(std::move(frame_));

	//Create measurements
	for (auto &match : mTracker->getMatches())
	{
		auto m = std::make_unique<FeatureMeasurement>(const_cast<Feature*>(&match.getFeature()), frame, match.getPosition(), match.getOctave(), match.getDescriptor());
		frame->getMeasurements().push_back(m.get());
		m->getFeature().getMeasurements().push_back(std::move(m));
	}

	//BA
	BundleAdjuster ba;
	ba.setMap(mMap.get());
	ba.setOutlierThreshold(2.5f);
	ba.setUseLocks(false);
	for (auto &framePtr : mMap->getKeyframes())
	{
		ba.addFrameToAdjust(*framePtr);
	}

	//ba.bundleAdjust();
	
}

void PlaneCalibSystem::doFullBA()
{
	//Set pose for reference frame
	Keyframe &refFrame = *mMap->getKeyframes()[0];

	Eigen::Vector3f basis1, basis2, basis3;
	basis3 = mCalib->getNormal().cast<float>();
	HomographyCalibrationError::GetBasis(basis3.data(), basis1, basis2);

	refFrame.mPose3DR(0, 0) = basis1[0];
	refFrame.mPose3DR(1, 0) = basis1[1];
	refFrame.mPose3DR(2, 0) = basis1[2];

	refFrame.mPose3DR(0, 1) = basis2[0];
	refFrame.mPose3DR(1, 1) = basis2[1];
	refFrame.mPose3DR(2, 1) = basis2[2];

	refFrame.mPose3DR(0, 2) = basis3[0];
	refFrame.mPose3DR(1, 2) = basis3[1];
	refFrame.mPose3DR(2, 2) = basis3[2];

	refFrame.mPose3DT = -refFrame.mPose3DR*basis3; //RefCenter = -R'*t = Normal (exactly one unit away from plane center) => t = -R*normal

	//Triangulate all features
	for (auto &feature : mMap->getFeatures())
	{

	}
}

} /* namespace dtslam */
