
#include "PlaneCalibSystem.h"
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>

#include "Keyframe.h"
#include "Profiler.h"
#include "PoseTracker.h"
#include "HomographyCalibration.h"
#include "HomographyEstimation.h"
#include "BundleAdjuster.h"
#include "CalibratedBundleAdjuster.h"
#include "PnpEstimation.h"
//#include "CeresUtils.h"
#include "FeatureIndexer.h"
#include "flags.h"

#include <random>

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

	mImageSize = keyframe->getImageSize();

    MYAPP_LOG << "Pyramid sizes: ";
	MYAPP_LOG << eutils::ToSize(mImageSize);
    for(int octave=1; octave<keyframe->getPyramid().getOctaveCount(); octave++)
		MYAPP_LOG << ", " << keyframe->getImage(octave).size();
	MYAPP_LOG << "\n";

	MYAPP_LOG << "SBI size: " << keyframe->getSBI().size() << "\n";


	//Init camera
	mCamera.init(mImageSize.cast<float>() / 2, Eigen::Vector2f(1,1), mImageSize);
	mCamera.getDistortionModel().init();

	//Normal
	mNormal = Eigen::Vector3f::Zero();

	//Reset map
	mMap.reset(new Map());

	//Reset tracker
	mTracker.reset(new PoseTracker());
	mTracker->init(mImageSize, keyframe->getOctaveCount());
	mTracker->setCamera(mCamera);

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

		FeatureGridIndexer<IndexedCvKeypoint> indexer;
		indexer.create(pkeyframe->getPyramid()[octave].size(), cv::Size2i(100, 100), 5);

		//cv::Size2i tileSize(scale*FLAGS_FrameKeypointGridSize, scale*FLAGS_FrameKeypointGridSize);
		//auto keypoints = FeatureGridIndexer<KeypointData>::ApplyNonMaximaSuppresion(pkeyframe->getKeypoints(octave), imageSize, tileSize, scale*16);
		auto keypoints = pkeyframe->getKeypoints(octave);
		auto descriptors = pkeyframe->getDescriptors(octave);
		
		for (uint i = 0; i < keypoints.size(); i++)
		{
			auto &keypoint = keypoints[i];
			void *descriptor = &descriptors(i, 0);
			indexer.addFeature(IndexedCvKeypoint(&keypoint, descriptor));
		}

		for (auto &kp : indexer)
		{
			mMap->createFeature(*pkeyframe, poseInv, Eigen::Vector2f(kp.keypoint->pt.x, kp.keypoint->pt.y), octave, (uchar*)kp.descriptor);
		}

		MYAPP_LOG << "Created " << keypoints.size() << " features in octave " << octave << "\n";
	}

	mTracker->resetTracking(mMap.get(), mMap->getKeyframes().back()->getPose());

	return true;
}

void PlaneCalibSystem::setMap(std::unique_ptr<Map> map)
{
	mMap = std::move(map);
	
	resetCalib();
}

void PlaneCalibSystem::resetCalib()
{
	auto &frame = *mMap->getKeyframes().back();
	mImageSize = frame.getImageSize();

	mTracker->init(mImageSize, frame.getOctaveCount());
	mTracker->resetTracking(mMap.get(), frame.getPose());

	std::vector<Eigen::Matrix3fr> allPoses;
	for (auto &frame : mMap->getKeyframes())
	{
		allPoses.push_back(frame->getPose());
	}

	//Calibrate
	mCamera.init(mImageSize.cast<float>() / 2, Eigen::Vector2f::Zero(), mImageSize);
	mCamera.getDistortionModel().init();
	//mCalib->calibrate(mHomographyP0, allPoses);
}

void PlaneCalibSystem::processImage(double timestamp, cv::Mat3b &imgColor, cv::Mat1b &imgGray)
{
	//Reset profiler for each image
	//Profiler::Instance().reset();

	mKeyframeAdded = false;

	if (mSuccesfulTrackCount < 20)
	{
		mTracker->mForceRansac = true;
	}
	else
	{
		mTracker->mForceRansac = false;
	}

	//Tracking
	bool trackingSuccesful=false;
	{
		//Read lock while tracking
		shared_lock<shared_mutex> lockRead(mMap->getMutex());

		//Track
		trackingSuccesful = mTracker->trackFrame(timestamp, imgColor, imgGray);
	}

	if (trackingSuccesful)
	{
		mSuccesfulTrackCount++;

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
		const float kThreshold = 60;
		const float kThresholdSq = kThreshold*kThreshold;
		std::vector<Eigen::Vector2f> cornersOriginal;
		cornersOriginal.push_back(Eigen::Vector2f(0, 0));
		cornersOriginal.push_back(Eigen::Vector2f(0, mTracker->getImageSize().y()));
		cornersOriginal.push_back(Eigen::Vector2f(mTracker->getImageSize().x(), 0));
		cornersOriginal.push_back(Eigen::Vector2f(mTracker->getImageSize().x(), mTracker->getImageSize().y()));

		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> corners;
		Eigen::Matrix3fr poseInv = mTracker->getCurrentPose2D().inverse();
		for (auto &p : cornersOriginal)
			corners.emplace_back(p, eutils::HomographyPoint(poseInv, p));

		bool add = true;
		if (mSuccesfulTrackCount > 20 && mTracker->getMatchInlierCount() > 100)
		{
			for (auto &frame_ : mMap->getKeyframes())
			{
				auto &frame = *frame_;
				float distSq = 0;
				for (auto &p : corners)
				{
					auto pp = eutils::HomographyPoint(frame.getPose(), p.second);
					distSq = std::max(distSq, (p.first - pp).squaredNorm());
				}

				if (distSq < kThresholdSq)
				{
					add = false;
					break;
				}
			}
		}
		else
		{
			add = false;
		}

		//Stop adding keyframes after calibrated BA
		if (mMap->getIs3DValid())
			add = false;

		//Add keyframe?
		if (add) 
		{
			createKeyframe();
			mKeyframeAdded = true;
			if (!mMap->getIs3DValid())
				doHomographyCalib(true);
		}
	}
	else
	{
		mSuccesfulTrackCount = 0;
	}
}

void PlaneCalibSystem::createKeyframe()
{
	std::unique_ptr<Keyframe> frame_(new Keyframe());
	Keyframe *frame = frame_.get();

	const TrackingFrame *trackerFrame = mTracker->getFrame();

	frame->init(*mTracker->getFrame());
	frame->setPose(mTracker->getCurrentPose2D());

	//Add keyframe to map
	mMap->addKeyframe(std::move(frame_));

	//Separate matches by octave
	std::vector<std::vector<const FeatureMatch *>> matchesByOctave;
	std::vector<std::vector<cv::KeyPoint>> keypointsByOctave;
	matchesByOctave.resize(trackerFrame->getOriginalPyramid().getOctaveCount());
	keypointsByOctave.resize(trackerFrame->getOriginalPyramid().getOctaveCount());
	for (auto &match : mTracker->getFrame()->getMatches())
	{
		const int scale = 1 << match.getOctave();
		
		cv::KeyPoint kp = match.getKeypoint();
		kp.octave = 0;
		kp.pt = eutils::ToCVPoint((match.getPosition() / scale).eval());
		kp.size /= scale;

		matchesByOctave[match.getOctave()].push_back(&match);
		keypointsByOctave[match.getOctave()].push_back(kp);
	}
	//Create descriptors
	cv::Ptr<cv::ORB> orb = cv::ORB::create(2000, 2, 1);
	orb->setEdgeThreshold(0);
	for (int octave = 0; octave < (int)matchesByOctave.size(); octave++)
	{
		if (keypointsByOctave.empty())
			continue;

		//ORB features
		cv::Mat1b descriptorBuffer;
		orb->detectAndCompute(frame->getImage(octave), cv::noArray(), keypointsByOctave[octave], descriptorBuffer, true);

		//Create measurement
		for (int i = 0; i < (int)matchesByOctave[octave].size(); i++)
		{
			auto &match = *matchesByOctave[octave][i];
			uchar *descriptor = &descriptorBuffer(i, 0);
			auto m = std::make_unique<FeatureMeasurement>(const_cast<Feature*>(&match.getFeature()), frame, match.getPosition(), octave, descriptor);

			//Save measurement
			frame->getMeasurements().push_back(m.get());
			m->getFeature().getMeasurements().push_back(std::move(m));
		}
	}

	//frame->freeSpace();
}

void PlaneCalibSystem::doHomographyBA()
{
	BundleAdjuster ba;
	ba.setUseLocks(false);
	ba.setMap(mMap.get());
	ba.setOutlierThreshold(3*mExpectedPixelNoiseStd);
	for (auto &framep : mMap->getKeyframes())
	{
		auto &frame = *framep;
		ba.addFrameToAdjust(frame);
	}

	ba.setOnlyDistortion(false);
	ba.setCamera(&mCamera);
	ba.bundleAdjust();

	//Calib
	//doHomographyCalib();

	//Update homography distortion with the new principal point
	//ba.setOnlyDistortion(true);
	//ba.setP0(Eigen::Vector2d(mK(0, 2), mK(1, 2)));
	//ba.bundleAdjust();

	//mHomographyP0 = ba.getP0().cast<float>();
	//mHomographyDistortion.setCoefficients(ba.getDistortion().cast<float>());
	//mActiveDistortion = &mHomographyDistortion;
}

void PlaneCalibSystem::doHomographyCalib(bool fixP0)
{
	std::vector<Eigen::Matrix3fr> allPoses;
	for (auto &frame : mMap->getKeyframes())
	{
		allPoses.push_back(frame->getPose());
	}

	//Calibrate
	mCalib->setUseNormalizedConstraints(mUseNormalizedConstraints);
	mCalib->setFixPrincipalPoint(fixP0);
	mCalib->initFromCamera(mCamera);
	mCalib->calibrate(allPoses);

	mCalib->updateCamera(mCamera);
	mNormal = mCalib->getNormal().cast<float>();
	//mK << 600, 0, 320, 0, 600, 240, 0, 0, 1;
	//mNormal << 0, 0, 1;
}

void PlaneCalibSystem::doFullBA()
{
	if (!mMap->getIs3DValid())
	{
		MYAPP_LOG << "Initializing metric reconstruction...\n";

		if (mUse3DGroundTruth)
		{
			mCamera = *mMap->mGroundTruthCamera;

			for (auto &pfeature : mMap->getFeatures())
			{
				auto &feature = *pfeature;
				feature.mPosition3D = feature.mGroundTruthPosition3D;
			}
			for (auto &framep : mMap->getKeyframes())
			{
				auto &frame = *framep;
				frame.mPose3DR = frame.mGroundTruthPose3DR;
				frame.mPose3DT = frame.mGroundTruthPose3DT;
			}
		}
		else
		{
			//Set pose for reference frame
			Keyframe &refFrame = *mMap->getKeyframes()[0];

			Eigen::Vector3f basis1, basis2, basis3;
			basis3 = mNormal;
			eutils::GetBasis(basis3, basis1, basis2);
			basis1.normalize();
			basis2.normalize();

			refFrame.mPose3DR(0, 0) = basis1[0];
			refFrame.mPose3DR(1, 0) = basis1[1];
			refFrame.mPose3DR(2, 0) = basis1[2];

			refFrame.mPose3DR(0, 1) = basis2[0];
			refFrame.mPose3DR(1, 1) = basis2[1];
			refFrame.mPose3DR(2, 1) = basis2[2];

			refFrame.mPose3DR(0, 2) = basis3[0];
			refFrame.mPose3DR(1, 2) = basis3[1];
			refFrame.mPose3DR(2, 2) = basis3[2];

			Eigen::Vector3f refCenter = -basis3;
			MYAPP_LOG << "Center of ref camera: " << refCenter.transpose() << "\n";
			refFrame.mPose3DT = -refFrame.mPose3DR*refCenter; //RefCenter = -R'*t = Normal (exactly one unit away from plane center) => t = -R*normal
			//refFrame.mPose3DR = refFrame.mGroundTruthPose3DR;
			//refFrame.mPose3DT = refFrame.mGroundTruthPose3DT;
			//refCenter = -refFrame.mPose3DR.transpose() * refFrame.mPose3DT;

			//auto k = mCamera.getK();
			//cv::Mat1f cvK(3, 3, const_cast<float*>(k.data()));

			//Triangulate all features
			for (auto &pfeature : mMap->getFeatures())
			{
				auto &feature = *pfeature;
		
				Eigen::Vector3f xn = mCamera.unprojectToWorld(feature.getPosition());
				Eigen::Vector3f xdir = refFrame.mPose3DR.transpose()*xn;
		
				//Intersect with plane
				//If point in line is x=a*t + b
				//and point in plane is dot(x,n)-d = 0
				//then t=(d-dot(b,n))/dot(a,n)
				//and x = a*(d-dot(b,n))/dot(a,n) + b
				//
				//Here b=refCenter, a=xdir, n=[0,0,1]', d=0
				feature.mPosition3D = refCenter - (refCenter[2]/xdir[2])*xdir; 
				feature.mPosition3D[2] = 0; //Just in case

				//Eigen::Vector3f xnt = refFrame.mPose3DR * feature.mPosition3D + refFrame.mPose3DT;
				//Eigen::Vector2f imagePosClean = mCamera.projectFromWorld(xnt);

				//Eigen::Vector3f xnt2 = refFrame.mPose3DR * feature.mGroundTruthPosition3D + refFrame.mPose3DT;
				//Eigen::Vector2f imagePosClean2 = mCamera.projectFromWorld(xnt2);
			}

			//Estimate frame positions
			for (auto &framep : mMap->getKeyframes())
			{
				auto &frame = *framep;

				//Skip ref frame
				if (&frame == &refFrame)
					continue;

				//Build constraints
				std::vector<Eigen::Vector3f> refPoints;
				std::vector<Eigen::Vector2f> imgPoints;
				std::vector<float> scales;
				for (auto &mp : frame.getMeasurements())
				{
					auto &m = *mp;

					refPoints.push_back(m.getFeature().mPosition3D);
					imgPoints.push_back(m.getPosition());
					scales.push_back((float)(1<<m.getOctave()));
				}

				//PnP
				PnPRansac ransac;
				ransac.setParams(3 * mExpectedPixelNoiseStd, 10, 100, (int)(0.99f * frame.getMeasurements().size()));
				ransac.setData(&refPoints, &imgPoints, &scales, &mCamera);
				ransac.doRansac();
				//MYAPP_LOG << "Frame pnp inlier count: " << ransac.getBestInlierCount() << "/" << matches.size() << "\n";
				frame.mPose3DR = ransac.getBestModel().first.cast<float>();
				frame.mPose3DT = ransac.getBestModel().second.cast<float>();

				//Refine
				int inlierCount;
				std::vector<MatchReprojectionErrors> errors;
				PnPRefiner refiner;
				refiner.setCamera(&mCamera);
				refiner.setOutlierThreshold(3 * mExpectedPixelNoiseStd);
				refiner.refinePose(refPoints, imgPoints, scales, frame.mPose3DR, frame.mPose3DT, inlierCount, errors);
				//Save
				//cv::Rodrigues(rvec, cvR);
		
				//frame.mPose3DR = mapR.cast<float>();
				//frame.mPose3DT[0] = (float)tvec(0, 0);
				//frame.mPose3DT[1] = (float)tvec(1, 0);
				//frame.mPose3DT[2] = (float)tvec(2, 0);
			}
		}
		mMap->setIs3DValid(true);
		auto poseH = mTracker->getCurrentPose2D();
		mTracker->resetTracking(mMap.get(), poseH);
	}

	//Camera params
	//distortion[1] += 0.03;
	//BAAAAA!!!
	CalibratedBundleAdjuster ba;
	ba.setUseLocks(false);
	ba.setFix3DPoints(mFix3DPoints);
	ba.setOutlierThreshold(3 * mExpectedPixelNoiseStd);
	ba.setCamera(&mCamera);
	//ba.setFixDistortion(true);
	//ba.setDistortion(Eigen::Vector2d(0.0935491, -0.157975));
	ba.setMap(mMap.get());
	for (auto &framep : mMap->getKeyframes())
	{
		ba.addFrameToAdjust(*framep);
	}
	ba.bundleAdjust();

	mMap->mCamera.reset(new CameraModel(mCamera));

	//Log
	//MatlabDataLog::Instance().AddValue("K", ba.getK());
	//MatlabDataLog::Instance().AddValue("Kold", mCalib->getK());
	//MatlabDataLog::Instance().AddValue("Nold", mCalib->getNormal());
	//for (auto &framep : mMap->getKeyframes())
	//{
	//	auto &frame = *framep;
	//	MatlabDataLog::Instance().AddCell("poseR", frame.mPose3DR);
	//	MatlabDataLog::Instance().AddCell("poseT", frame.mPose3DT);
	//	MatlabDataLog::Instance().AddCell("poseH", frame.getPose());

	//	MatlabDataLog::Instance().AddCell("posM");
	//	MatlabDataLog::Instance().AddCell("posRt");
	//	MatlabDataLog::Instance().AddCell("posH");
	//	for (auto &mp : frame.getMeasurements())
	//	{
	//		auto &m = *mp;

	//		MatlabDataLog::Instance().AddValueToCell("posM", m.getPosition());

	//		Eigen::Vector3f mm;
	//		Eigen::Vector2f m2;

	//		mm = ba.getK().cast<float>() * (frame.mPose3DR*m.getFeature().mPosition3D + frame.mPose3DT);
	//		m2 << mm[0] / mm[2], mm[1] / mm[2];

	//		MatlabDataLog::Instance().AddValueToCell("posRt", m2);

	//		Eigen::Vector3f p; p << m.getFeature().getPosition()[0], m.getFeature().getPosition()[1], 1;
	//		mm = frame.getPose() * p;
	//		m2 << mm[0] / mm[2], mm[1] / mm[2];

	//		MatlabDataLog::Instance().AddValueToCell("posH", m2);

	//	}
	//}
}

void PlaneCalibSystem::doValidationBA()
{
	//BAAAAA!!!
	CalibratedBundleAdjuster ba;
	ba.setUseLocks(false);
	ba.setFixPrincipalPoint(true);
	ba.setFixDistortion(true);
	ba.setFixFocalLengths(true);
	ba.setFix3DPoints(true);
	ba.setOutlierThreshold(3 * mExpectedPixelNoiseStd);
	ba.setCamera(&mCamera);
	ba.setMap(mMap.get());
	for (auto &framep : mMap->getKeyframes())
	{
		ba.addFrameToAdjust(*framep);
	}
	ba.bundleAdjust();
}

} 
