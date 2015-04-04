
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

	//Distortion
	mCameraDistortion.reset(new RadialCameraDistortionModel());
	mCameraDistortion->init(Eigen::Vector2f::Zero(), imageSize);

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
	
	auto &frame = *mMap->getKeyframes().back();
	mTracker->init(frame.getImageSize(), frame.getOctaveCount());
	mTracker->resetTracking(mMap.get(), frame.getPose());

	std::vector<Eigen::Matrix3fr> allPoses;
	for (auto &frame : mMap->getKeyframes())
	{
		allPoses.push_back(frame->getPose());
	}

	//Calibrate
	mCalib->calibrate(allPoses, mTracker->getImageSize());
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

void PlaneCalibSystem::doHomographyBA()
{
	BundleAdjuster ba;
	for (auto &framep : mMap->getKeyframes())
	{
		auto &frame = *framep;
		ba.addFrameToAdjust(frame);
	}
	ba.setUseLocks(false);
	ba.setOutlierThreshold(3);
	ba.setMap(mMap.get());
	ba.bundleAdjust();
}

void PlaneCalibSystem::doFullBA()
{
	//Set pose for reference frame
	Keyframe &refFrame = *mMap->getKeyframes()[0];

	Eigen::Vector3f basis1, basis2, basis3;
	basis3 = mCalib->getNormal().cast<float>();
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

	Eigen::Vector3f refCenter = basis3;
	refFrame.mPose3DT = -refFrame.mPose3DR*refCenter; //RefCenter = -R'*t = Normal (exactly one unit away from plane center) => t = -R*normal

	//Invert K
	Eigen::Matrix3fr Kinv;
	float fi = 1.0f / mCalib->getK()(0, 0);
	Kinv << fi, 0, -fi*mCalib->getK()(0, 2), 0, fi, -fi*mCalib->getK()(1, 2), 0, 0, 1;

	//Triangulate all features
	for (auto &pfeature : mMap->getFeatures())
	{
		auto &feature = *pfeature;
		
		Eigen::Vector3f m;
		m << feature.getPosition()[0], feature.getPosition()[1], 1;

		Eigen::Vector3f xn = Kinv * m;

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

		//Check
		//Eigen::Vector3f tl = xdir.cross(feature.mPosition3D-refCenter);
		//float tp = feature.mPosition3D[2];
		Eigen::Vector3f mm = mCalib->getK() * (refFrame.mPose3DR*feature.mPosition3D + refFrame.mPose3DT);
		Eigen::Vector2f m2;
		m2 << mm[0] / mm[2], mm[1] / mm[2];

		float diff = (m2 - feature.getPosition()).norm();
		MatlabDataLog::Instance().AddValue("diffTest", diff);
	}

	//Estimate frame positions
	cv::Mat1f cvK(3,3,const_cast<float*>(mCalib->getK().data()));
	for (auto &framep : mMap->getKeyframes())
	{
		auto &frame = *framep;
		
		//Skip ref frame
		if (&frame == &refFrame)
			continue;

		//Build constraints
		std::vector<cv::Point3f> cvWorldPoints;
		std::vector<cv::Point2f> cvImagePoints;
		for (auto &mp : frame.getMeasurements())
		{
			auto &m = *mp;
			cvImagePoints.push_back(cv::Point2f(m.getPosition()[0], m.getPosition()[1]));
			cvWorldPoints.push_back(cv::Point3f(m.getFeature().mPosition3D[0], m.getFeature().mPosition3D[1], m.getFeature().mPosition3D[2]));
		}

		//PnP
		cv::Vec3d rvec, tvec;
		cv::Matx33d cvR;
		Eigen::Map<Eigen::Matrix3dr> mapR(cvR.val);

		mapR = refFrame.mPose3DR.cast<double>();
		cv::Rodrigues(cvR, rvec);
		tvec[0] = refFrame.mPose3DT[0];
		tvec[1] = refFrame.mPose3DT[1];
		tvec[2] = refFrame.mPose3DT[2];

		//cv::Vec4f dist(0, 0, 0, 0);
		cv::solvePnP(cvWorldPoints, cvImagePoints, cvK, cv::noArray(), rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);

		//Save
		cv::Rodrigues(rvec, cvR);
		
		frame.mPose3DR = mapR.cast<float>();
		frame.mPose3DT[0] = (float)tvec[0];
		frame.mPose3DT[1] = (float)tvec[1];
		frame.mPose3DT[2] = (float)tvec[2];
	}

	//BAAAAA!!!
	CalibratedBundleAdjuster ba;
	ba.setUseLocks(false);
	ba.setOutlierThreshold(3);
	ba.setK(mCalib->getK().cast<double>());
	ba.setMap(mMap.get());
	for (auto &framep : mMap->getKeyframes())
	{
		ba.addFrameToAdjust(*framep);
	}
	//ba.addFrameToAdjust(**mMap->getKeyframes().begin());
	ba.bundleAdjust();
	MatlabDataLog::Instance().AddValue("K", ba.getK());

	//Log
	MatlabDataLog::Instance().AddValue("Kold", mCalib->getK());
	MatlabDataLog::Instance().AddValue("Nold", mCalib->getNormal());
	for (auto &framep : mMap->getKeyframes())
	{
		auto &frame = *framep;
		MatlabDataLog::Instance().AddCell("poseR", frame.mPose3DR);
		MatlabDataLog::Instance().AddCell("poseT", frame.mPose3DT);
		MatlabDataLog::Instance().AddCell("poseH", frame.getPose());

		MatlabDataLog::Instance().AddCell("posM");
		MatlabDataLog::Instance().AddCell("posRt");
		MatlabDataLog::Instance().AddCell("posH");
		for (auto &mp : frame.getMeasurements())
		{
			auto &m = *mp;

			MatlabDataLog::Instance().AddValueToCell("posM", m.getPosition());

			Eigen::Vector3f mm;
			Eigen::Vector2f m2;

			mm = ba.getK().cast<float>() * (frame.mPose3DR*m.getFeature().mPosition3D + frame.mPose3DT);
			m2 << mm[0] / mm[2], mm[1] / mm[2];

			MatlabDataLog::Instance().AddValueToCell("posRt", m2);

			Eigen::Vector3f p; p << m.getFeature().getPosition()[0], m.getFeature().getPosition()[1], 1;
			mm = frame.getPose() * p;
			m2 << mm[0] / mm[2], mm[1] / mm[2];

			MatlabDataLog::Instance().AddValueToCell("posH", m2);

		}
	}
}

} /* namespace dtslam */
