
#include "PlaneCalibSystem.h"
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>
#include <ceres/rotation.h>

#include "Keyframe.h"
#include "Profiler.h"
#include "PoseTracker.h"
#include "HomographyCalibration.h"
#include "HomographyEstimation.h"
#include "BundleAdjuster.h"
#include "CalibratedBundleAdjuster.h"
#include "PnpEstimation.h"
//#include "CeresUtils.h"
//#include "FeatureGridIndexer.h"
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

	Eigen::Vector2i imageSize = keyframe->getImageSize();

    MYAPP_LOG << "Pyramid sizes: ";
	MYAPP_LOG << eutils::ToSize(imageSize);
    for(int octave=1; octave<keyframe->getPyramid().getOctaveCount(); octave++)
		MYAPP_LOG << ", " << keyframe->getImage(octave).size();
	MYAPP_LOG << "\n";

	MYAPP_LOG << "SBI size: " << keyframe->getSBI().size() << "\n";

	//Distortion
	mHomographyP0 = imageSize.cast<float>() / 2;
	mHomographyDistortion.init(Eigen::Vector2f::Zero(), imageSize);
	mCamera.getDistortionModel().init(Eigen::Vector2f::Zero(), 1e10);

	mCamera.init(0,0,0,0,imageSize[0],imageSize[1]);
	mNormal = Eigen::Vector3f::Zero();

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

void PlaneCalibSystem::setMap(std::unique_ptr<Map> map)
{
	mMap = std::move(map);
	
	resetCalib();
}

void PlaneCalibSystem::resetCalib()
{
	auto &frame = *mMap->getKeyframes().back();
	mTracker->init(frame.getImageSize(), frame.getOctaveCount());
	mTracker->resetTracking(mMap.get(), frame.getPose());

	std::vector<Eigen::Matrix3fr> allPoses;
	for (auto &frame : mMap->getKeyframes())
	{
		allPoses.push_back(frame->getPose());
	}

	//Calibrate
	mHomographyP0 = frame.getImageSize().cast<float>() / 2;
	mHomographyDistortion.init(Eigen::Vector2f::Zero(), frame.getImageSize());
	//mCalib->calibrate(mHomographyP0, allPoses);
}

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
		const float kThreshold = 60;
		const float kThresholdSq = kThreshold*kThreshold;
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
				distSq = std::max(distSq,(p.first - pp).squaredNorm());
			}

			if (distSq < kThresholdSq)
			{
				add = false;
				break;
			}
		}

		if (add)
		{
			createKeyframe();

			//doHomographyCalib();
		}
	}
}

void PlaneCalibSystem::createKeyframe()
{
	std::unique_ptr<Keyframe> frame_(new Keyframe(*mTracker->getFrame()));
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

	frame->freeSpace();
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
	ba.setP0(mHomographyP0.cast<double>());
	ba.setDistortion(mHomographyDistortion.getCoefficients().cast<double>());
	ba.bundleAdjust();

	mHomographyP0 = ba.getP0().cast<float>();
	mHomographyDistortion.setCoefficients(ba.getDistortion().cast<float>());

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
	mCalib->calibrate(mHomographyP0, allPoses);
	mCamera.setFromK(mCalib->getK());
	mNormal = mCalib->getNormal().cast<float>();
	//mK << 600, 0, 320, 0, 600, 240, 0, 0, 1;
	//mNormal << 0, 0, 1;

	float fx2 = mCamera.getFx()*mCamera.getFx();
	Eigen::Vector2f distortion;
	distortion << mHomographyDistortion.getCoefficients()[0] * fx2, mHomographyDistortion.getCoefficients()[1] * fx2*fx2;
	MYAPP_LOG << "Transformed distortion: " << distortion[0] << ", " << distortion[1] << "\n";

	mCamera.getDistortionModel().setCoefficients(distortion);
}

void PlaneCalibSystem::doFullBA()
{
	Eigen::Matrix3fr k;
	Eigen::Vector2f distortion;
	
	MYAPP_LOG << "Initializing metric reconstruction...\n";

	if (mUse3DGroundTruth)
	{
		k = mMap->mGroundTruthCamera->getK();
		distortion = mMap->mGroundTruthCamera->getDistortionModel().getCoefficients();

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
		k = mCamera.getK();
		distortion = mCamera.getDistortionModel().getCoefficients();

		//Set pose for reference frame
		Keyframe &refFrame = *mMap->getKeyframes()[0];

		Eigen::Vector3f basis1, basis2, basis3;
		basis3 = mNormal;
		HomographyCalibrationError::GetBasis(basis3.data(), basis1, basis2);
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
		refFrame.mPose3DT = -refFrame.mPose3DR*refCenter; //RefCenter = -R'*t = Normal (exactly one unit away from plane center) => t = -R*normal
		//refFrame.mPose3DR = refFrame.mGroundTruthPose3DR;
		//refFrame.mPose3DT = refFrame.mGroundTruthPose3DT;
		//refCenter = -refFrame.mPose3DR.transpose() * refFrame.mPose3DT;

		cv::Mat1f cvK(3, 3, const_cast<float*>(k.data()));

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
			std::vector<FeatureMatch> matches;
			//std::vector<cv::Point3f> cvWorldPoints;
			//std::vector<cv::Point2f> cvImagePoints;
			for (auto &mp : frame.getMeasurements())
			{
				auto &m = *mp;
				//Eigen::Vector2f mu = mHomographyDistortion.undistortPoint(m.getPosition() - mHomographyP0) + mHomographyP0;
				//cvImagePoints.push_back(cv::Point2f(mu[0], mu[1]));
				//cvWorldPoints.push_back(cv::Point3f(m.getFeature().mPosition3D[0], m.getFeature().mPosition3D[1], m.getFeature().mPosition3D[2]));
				
				matches.push_back(FeatureMatch(&m, m.getOctave(), m.getPosition(), m.getDescriptor().data(), 0));
			}

			//PnP
			//cv::Mat1d rvec(3,1), tvec(3,1);
			//cv::Matx33d cvR;
			//Eigen::Map<Eigen::Matrix3dr> mapR(cvR.val);

			//mapR = refFrame.mPose3DR.cast<double>();
			//cv::Rodrigues(cvR, rvec);
			//tvec(0, 0) = refFrame.mPose3DT[0];
			//tvec(1, 0) = refFrame.mPose3DT[1];
			//tvec(2, 0) = refFrame.mPose3DT[2];

			//cv::Vec4f dist(0, 0, 0, 0);
			//cv::solvePnPRansac(cvWorldPoints, cvImagePoints, cvK, cv::noArray(), rvec, tvec, true, 100, 3*mExpectedPixelNoiseStd, 1.0 - std::numeric_limits<float>::epsilon());
			//cv::solvePnP(cvWorldPoints, cvImagePoints, cvK, cv::noArray(), rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);

			PnPRansac ransac;
			ransac.setParams(3 * mExpectedPixelNoiseStd, 10, 100, (int)(0.9f * frame.getMeasurements().size()));
			ransac.setData(matches, &mCamera);
			ransac.doRansac();
			//MYAPP_LOG << "Frame pnp inlier count: " << ransac.getBestInlierCount() << "/" << matches.size() << "\n";
			frame.mPose3DR = ransac.getBestModel().first.cast<float>();
			frame.mPose3DT = ransac.getBestModel().second.cast<float>();

			//Save
			//cv::Rodrigues(rvec, cvR);
		
			//frame.mPose3DR = mapR.cast<float>();
			//frame.mPose3DT[0] = (float)tvec(0, 0);
			//frame.mPose3DT[1] = (float)tvec(1, 0);
			//frame.mPose3DT[2] = (float)tvec(2, 0);
		}
	}
	mMap->setIs3DValid(true);

	//BAAAAA!!!
	CalibratedBundleAdjuster ba;
	ba.setUseLocks(false);
	ba.setFix3DPoints(mFix3DPoints);
	ba.setOutlierThreshold(3 * mExpectedPixelNoiseStd);
	ba.setDistortion(distortion.cast<double>());
	ba.setK(k.cast<double>());
	ba.setMap(mMap.get());
	for (auto &framep : mMap->getKeyframes())
	{
		ba.addFrameToAdjust(*framep);
	}
	ba.bundleAdjust();

	mCamera.getDistortionModel().setCoefficients(ba.getDistortion().cast<float>());
	mCamera.setFromK(ba.getK().cast<float>());

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
	Eigen::Matrix3fr k;
	Eigen::Vector2f distortion;

	k = mCamera.getK();
	distortion = mCamera.getDistortionModel().getCoefficients();

	//BAAAAA!!!
	CalibratedBundleAdjuster ba;
	ba.setUseLocks(false);
	ba.setFixCalib(true);
	ba.setFix3DPoints(true);
	ba.setOutlierThreshold(3 * mExpectedPixelNoiseStd);
	ba.setDistortion(distortion.cast<double>());
	ba.setK(k.cast<double>());
	ba.setMap(mMap.get());
	for (auto &framep : mMap->getKeyframes())
	{
		ba.addFrameToAdjust(*framep);
	}
	ba.bundleAdjust();
}

} 
