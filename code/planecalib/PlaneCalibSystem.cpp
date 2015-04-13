
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
	mCameraDistortion.init(Eigen::Vector2f::Zero(), imageSize);
	mActiveDistortion = &mHomographyDistortion;

	mK = Eigen::Matrix3fr::Zero();
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
	mCalib->calibrate(mHomographyP0, allPoses);
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
	mActiveDistortion = &mHomographyDistortion;

	//Calib
	doHomographyCalib();

	//Update homography distortion with the new principal point
	//ba.setOnlyDistortion(true);
	//ba.setP0(Eigen::Vector2d(mK(0, 2), mK(1, 2)));
	//ba.bundleAdjust();

	//mHomographyP0 = ba.getP0().cast<float>();
	//mHomographyDistortion.setCoefficients(ba.getDistortion().cast<float>());
	//mActiveDistortion = &mHomographyDistortion;
}

void PlaneCalibSystem::doHomographyCalib()
{
	std::vector<Eigen::Matrix3fr> allPoses;
	for (auto &frame : mMap->getKeyframes())
	{
		allPoses.push_back(frame->getPose());
	}

	//Calibrate
	mCalib->calibrate(mHomographyP0, allPoses);
	mK = mCalib->getK().cast<float>();
	mNormal = mCalib->getNormal().cast<float>();

	float fx2 = mK(0, 0)*mK(0, 0);
	MYAPP_LOG << "Translated distortion: " << mHomographyDistortion.getCoefficients()[0] * fx2 << ", " << mHomographyDistortion.getCoefficients()[1] * fx2*fx2 << "\n";
}

void PlaneCalibSystem::doFullBA()
{
	float fx2 = mK(0, 0) * mK(0, 0);
	Eigen::Vector2f distortion(mHomographyDistortion.getCoefficients()[0] * fx2, mHomographyDistortion.getCoefficients()[1]*fx2*fx2);

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

	//Invert K
	Eigen::Matrix3fr Kinv;
	float fxi = 1.0f / mK(0, 0);
	float fyi = 1.0f / mK(1, 1);
	Kinv << fxi, 0, -fxi*mK(0, 2), 0, fyi, -fyi*mK(1, 2), 0, 0, 1;

	//Triangulate all features
	for (auto &pfeature : mMap->getFeatures())
	{
		auto &feature = *pfeature;
		
		Eigen::Vector2f m = mHomographyDistortion.undistortPoint(feature.getPosition() - mHomographyP0) + mHomographyP0;

		Eigen::Vector3f xn = Kinv * m.homogeneous();

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
		//Eigen::Vector3f mm = mCalib->getK() * (refFrame.mPose3DR*feature.mPosition3D + refFrame.mPose3DT);
		//Eigen::Vector2f m2;
		//m2 << mm[0] / mm[2], mm[1] / mm[2];

		//float diff = (m2 - feature.getPosition()).norm();
		//MatlabDataLog::Instance().AddValue("diffTest", diff);
	}

	//Estimate frame positions
	cv::Mat1f cvK(3,3,const_cast<float*>(mK.data()));
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
			Eigen::Vector2f mu = mHomographyDistortion.undistortPoint(m.getPosition() - mHomographyP0) + mHomographyP0;
			cvImagePoints.push_back(cv::Point2f(mu[0], mu[1]));
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
	mMap->setIs3DValid(true);

	//BAAAAA!!!
	CalibratedBundleAdjuster ba;
	ba.setUseLocks(false);
	ba.setOutlierThreshold(3 * mExpectedPixelNoiseStd);
	ba.setDistortion(distortion.cast<double>());
	ba.setK(mK.cast<double>());
	ba.setMap(mMap.get());
	for (auto &framep : mMap->getKeyframes())
	{
		ba.addFrameToAdjust(*framep);
	}
	//ba.addFrameToAdjust(**mMap->getKeyframes().begin());
	ba.bundleAdjust();

	mCameraDistortion.setCoefficients(ba.getDistortion().cast<float>());
	mActiveDistortion = &mCameraDistortion;

	mK = ba.getK().cast<float>();

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

void PlaneCalibSystem::generateSyntheticMap(const Eigen::Matrix3fr &k, const Eigen::Vector2f &distortionCoeffs, const Eigen::Vector2i &imageSize, float measurementNoiseStd)
{
	std::random_device rd;
	std::unique_ptr<Map> newMap(new Map());

	mExpectedPixelNoiseStd = std::max(0.3f,measurementNoiseStd);

	cv::Mat3b nullImg3(imageSize[1], imageSize[0]);
	cv::Mat1b nullImg1(imageSize[1], imageSize[0]);
	Eigen::Matrix<uchar, 1, 32> nullDescr;
	nullDescr.setZero();

	//Invert
	Eigen::Vector2f imagePlaneSize(imageSize[0] / k(1, 1), imageSize[1] / k(2, 2));
	RadialCameraDistortionModel distortion;
	distortion.init(distortionCoeffs, imagePlaneSize);

	Eigen::Matrix3fr kinv = k.inverse();

	//Features
	const int kFeatureCount = 1000;
	std::uniform_real_distribution<float> distributionx(0,imageSize[0]);
	std::uniform_real_distribution<float> distributiony(0, imageSize[1]);
	for (int i = 0; i < kFeatureCount; i++)
	{
		std::unique_ptr<Feature> newFeature(new Feature());
		
		//Get position
		Eigen::Vector2f imagePos(distributionx(rd), distributiony(rd));
		Eigen::Vector2f xd = (kinv*imagePos.homogeneous()).eval().hnormalized();
		Eigen::Vector2f xn = distortion.undistortPoint(xd);

		newFeature->mPosition3D = Eigen::Vector3f(xn[0],xn[1],0);
		newFeature->setPosition(imagePos);

		newMap->addFeature(std::move(newFeature));
	}

	//Frame poses
	std::vector<Eigen::Matrix3fr> posesR;
	std::vector<Eigen::Vector3f> posesCenter;

	posesR.push_back(Eigen::Matrix3fr::Identity());
	posesCenter.push_back(Eigen::Vector3f(0,0,-1));

	for (float alpha = -45; alpha < 45; alpha += 10)
	{
		float rad = alpha*M_PI/180;
		posesR.push_back(eutils::RotationY(rad));
		posesCenter.push_back(Eigen::Vector3f(1*sin(rad), 0, -1*cos(rad)));
	}
	for (float alpha = -45; alpha < 45; alpha += 10)
	{
		float rad = alpha*M_PI / 180;
		posesR.push_back(eutils::RotationX(rad));
		posesCenter.push_back(Eigen::Vector3f(0, -1 * sin(rad), -1 * cos(rad)));
	}

	//Set reference frame
	const int kRefFrameIdx = 0;
	if (kRefFrameIdx != 0)
	{
		std::swap(posesR[0], posesR[kRefFrameIdx]);
		std::swap(posesCenter[0], posesCenter[kRefFrameIdx]);
	}

	////Matlab log
	//Eigen::MatrixX3f points(newMap->getFeatures().size(),3);
	//for (int i = 0; i < points.rows(); i++)
	//{
	//	points.row(i) = newMap->getFeatures()[i]->mPosition3D.transpose();
	//}
	//MatlabDataLog::Instance().AddValue("K", k);
	//MatlabDataLog::Instance().AddValue("X",points.transpose());
	//for (int i = 0; i < posesR.size(); i++)
	//{
	//	MatlabDataLog::Instance().AddCell("R",posesR[i]);
	//	MatlabDataLog::Instance().AddCell("center", posesCenter[i]);
	//}

	//Frames
	std::normal_distribution<float> error_distribution(0, measurementNoiseStd);
	for (int i = 0; i < (int)posesR.size(); i++)
	{
		std::unique_ptr<Keyframe> newFrame(new Keyframe());

		newFrame->init(nullImg3, nullImg1);

		//Stats
		std::vector<float> distortionError;
		std::vector<float> noiseError;

		//Points
		std::vector<cv::Point2f> refPoints, imgPoints;
		for (int j = 0; j< (int)newMap->getFeatures().size(); j++)
		{
			auto &feature = *newMap->getFeatures()[j];

			//Project
			Eigen::Vector2f xn = (posesR[i] * feature.mPosition3D - posesR[i] * posesCenter[i]).eval().hnormalized();
			Eigen::Vector2f xd = distortion.distortPoint(xn);
			Eigen::Vector2f imagePosClean = (k*xd.homogeneous()).eval().hnormalized(); //.unaryExpr(std::roundf)
			Eigen::Vector2f imagePosNoDistortion = (k*xn.homogeneous()).eval().hnormalized(); //.unaryExpr(std::roundf)
			Eigen::Vector2f noise(error_distribution(rd), error_distribution(rd));
			Eigen::Vector2f imagePos = imagePosClean + noise;

			//Skip if outside of image
			if (imagePos[0] < 0 || imagePos[1] < 0 || imagePos[0] > imageSize[0] || imagePos[1] > imageSize[1])
				continue;

			distortionError.push_back((imagePosClean-imagePosNoDistortion).norm());
			noiseError.push_back(noise.norm());

			//Position
			if (i == 0)
				feature.setPosition(imagePos);

			//Measurement
			std::unique_ptr<FeatureMeasurement> m(new FeatureMeasurement(&feature, newFrame.get(), imagePos, 0, nullDescr.data()));
			newFrame->getMeasurements().push_back(m.get());
			feature.getMeasurements().push_back(std::move(m));

			//Save match
			refPoints.push_back(eutils::ToCVPoint(feature.getPosition()));
			imgPoints.push_back(eutils::ToCVPoint(imagePos));
		}

		//Write stats
		Eigen::Map<Eigen::ArrayXf> _distortionError(distortionError.data(), distortionError.size());
		Eigen::Map<Eigen::ArrayXf> _noiseError(noiseError.data(), noiseError.size());
		MYAPP_LOG << "Frame " << i << "\n";
		MYAPP_LOG << "  Measurement count: " << newFrame->getMeasurements().size() << "\n";
		MYAPP_LOG << "  Max distortion error: " << _distortionError.maxCoeff() << "\n";
		MYAPP_LOG << "  Max noise error: " << _noiseError.maxCoeff() << "\n";

		//Get homography
		Eigen::Matrix<uchar, Eigen::Dynamic, 1> mask(refPoints.size());
		cv::Mat1b mask_cv(refPoints.size(), 1, mask.data());

		cv::Mat H;
		H = cv::findHomography(refPoints, imgPoints, cv::RANSAC, 3 * mExpectedPixelNoiseStd, mask_cv);
		cv::Matx33f cvH = cv::Matx33f::eye();
		if (!H.empty())
		{
			cvH = H;
		}
		else
		{
			MYAPP_LOG << "findHomography failed \n";
		}

		//Refine
		HomographyEstimation hest;
		std::vector<bool> inliersVec;
		std::vector<int> octaveVec(imgPoints.size(), 0);
		cvH = hest.estimateCeres(cvH, imgPoints, refPoints, octaveVec, 3 * mExpectedPixelNoiseStd, inliersVec);

		//Set final
		newFrame->setPose(eutils::FromCV(cvH));

		//Add
		newMap->addKeyframe(std::move(newFrame));
	}

	setMap(std::move(newMap));
}

} 
