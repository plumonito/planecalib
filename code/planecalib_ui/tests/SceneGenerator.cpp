#include "SceneGenerator.h"
#include "planecalib/Map.h"
#include "matio.h"
#include "planecalib/CameraModel.h"
#include "planecalib/HomographyEstimation.h"
#include <opencv2/calib3d.hpp>
#include "planecalib/HomographyCalibration.h"

namespace planecalib
{

std::unique_ptr<Map> SceneGenerator::generateSyntheticMap(const CameraModel &camera)
{
	//Frame poses
	std::vector<Eigen::Matrix3fr> posesR;
	std::vector<Eigen::Vector3f> posesCenter;

	posesR.push_back(Eigen::Matrix3fr::Identity());
	posesCenter.push_back(Eigen::Vector3f(0, 0, -1));

	for (float dx = -0.5; dx <= 0.5; dx += 0.25)
		for (float dy = -0.5; dy <= 0.5; dy += 0.25)
		{
			if (dx == 0 || dy == 0)
				continue;
			//posesR.push_back(Eigen::Matrix3fr::Identity());
			//posesCenter.push_back(Eigen::Vector3f(dx, dy, -1));
		}
	for (float alpha = -45; alpha < 45; alpha += 10)
	{
		float rad = alpha*M_PI / 180;
		posesR.push_back(eutils::RotationY(rad));
		posesCenter.push_back(Eigen::Vector3f(1 * sin(rad), 0, -1 * cos(rad)));
	}
	for (float alpha = -45; alpha < 45; alpha += 10)
	{
		float rad = alpha*M_PI / 180;
		posesR.push_back(eutils::RotationX(rad));
		posesCenter.push_back(Eigen::Vector3f(0, -1 * sin(rad), -1 * cos(rad)));
	}

	return generateFromPoses(camera, posesR, posesCenter);
}

std::unique_ptr<Map> SceneGenerator::generateRandomPoses(const CameraModel &camera, int frameCount)
{
	//Frame poses
	std::vector<Eigen::Matrix3fr> posesR;
	std::vector<Eigen::Vector3f> posesCenter;

	posesR.push_back(Eigen::Matrix3fr::Identity());
	posesCenter.push_back(Eigen::Vector3f(0, 0, -1));

	std::uniform_real_distribution<float> uniformTarget(-0.5f, +0.5f);
	std::uniform_real_distribution<float> uniformCenterXY(-0.8f, +0.8f);
	std::uniform_real_distribution<float> uniformCenterZ(-1.0f, -0.5f);
	std::uniform_real_distribution<float> uniformAngle(0.0f, 2*M_PI);

	for (int i = 0; i < frameCount; i++)
	{
		//Choose a point to look at
		Eigen::Vector3f lookTarget(uniformTarget(mRandomDevice), uniformTarget(mRandomDevice), 0);

		//Point to define up vector
		Eigen::Vector3f upTarget(uniformTarget(mRandomDevice), uniformTarget(mRandomDevice), 0);

		//Choose camera center
		Eigen::Vector3f center(uniformCenterXY(mRandomDevice), uniformCenterXY(mRandomDevice), uniformCenterZ(mRandomDevice));

		//Build rotation
		Eigen::Matrix3fr R;
		Eigen::Vector3f a, b, c;
		c = (lookTarget - center).normalized();
		HomographyCalibrationError::GetBasis(c.data(), a, b);
		
		Eigen::AngleAxisf aa(uniformAngle(mRandomDevice), c);
		a = (aa*a).normalized();
		b = (aa*b).normalized();

		R.col(0) = a;
		R.col(1) = b;
		R.col(2) = c;
		//R.col(1) = (lookTarget - R.col(2).dot(lookTarget)*R.col(2)).normalized();
		assert(abs(R.col(2).dot(R.col(1))) < 1e-6);
		assert(abs(R.col(2).dot(R.col(0))) < 1e-6);
		assert(abs(R.col(1).dot(R.col(0))) < 1e-6);
		//R.col(0) = R.col(1).cross(R.col(2));

		posesR.push_back(R.transpose());
		posesCenter.push_back(center);
	}

	return generateFromPoses(camera, posesR, posesCenter);
}
std::unique_ptr<Map> SceneGenerator::generateFromPoses(const CameraModel &camera, const std::vector<Eigen::Matrix3fr> &posesR, const std::vector<Eigen::Vector3f> &posesCenter)
{
	std::unique_ptr<Map> newMap(new Map());

	cv::Mat3b nullImg3(camera.getImageSize()[1], camera.getImageSize()[0]);
	cv::Mat1b nullImg1(camera.getImageSize()[1], camera.getImageSize()[0]);
	Eigen::Matrix<uchar, 1, 32> nullDescr;
	nullDescr.setZero();

	float expectedPixelNoiseStd = std::max(0.3f, mNoiseStd);

	//Features
	const int kFeatureCount = 1000;
	std::uniform_real_distribution<float> distributionx(0, camera.getImageSize()[0]);
	std::uniform_real_distribution<float> distributiony(0, camera.getImageSize()[1]);
	for (int i = 0; i < kFeatureCount; i++)
	{
		std::unique_ptr<Feature> newFeature(new Feature());

		//Get position
		Eigen::Vector2f imagePos(distributionx(mRandomDevice), distributiony(mRandomDevice));
		Eigen::Vector2f xn = camera.unprojectToWorld(imagePos).hnormalized();

		newFeature->mGroundTruthPosition3D = Eigen::Vector3f(xn[0], xn[1], 0);
		newFeature->mPosition3D = newFeature->mGroundTruthPosition3D;
		newFeature->setPosition(imagePos);

		newMap->addFeature(std::move(newFeature));
	}

	//Matlab log of poses
	Eigen::MatrixX3f points(newMap->getFeatures().size(), 3);
	for (int i = 0; i < points.rows(); i++)
	{
		points.row(i) = newMap->getFeatures()[i]->mPosition3D.transpose();
	}
	MatlabDataLog::Instance().AddValue("K", camera.getK());
	MatlabDataLog::Instance().AddValue("X", points.transpose());
	for (int i = 0; i < posesR.size(); i++)
	{
		MatlabDataLog::Instance().AddCell("R", posesR[i]);
		MatlabDataLog::Instance().AddCell("center", posesCenter[i]);
	}
	//return std::unique_ptr<Map>();

	//Frames
	std::normal_distribution<float> error_distribution(0, mNoiseStd);
	for (int i = 0; i < (int)posesR.size(); i++)
	{
		std::unique_ptr<Keyframe> newFrame(new Keyframe());

		newFrame->init(nullImg3, nullImg1);
		
		newFrame->mGroundTruthPose3DR = posesR[i];
		newFrame->mGroundTruthPose3DT = -posesR[i]*posesCenter[i];

		//Stats
		std::vector<float> distortionError;
		std::vector<float> noiseError;

		//Points
		std::vector<cv::Point2f> refPoints, imgPoints;
		for (int j = 0; j< (int)newMap->getFeatures().size(); j++)
		{
			auto &feature = *newMap->getFeatures()[j];

			//Project
			Eigen::Vector3f xn = posesR[i] * feature.mPosition3D - posesR[i] * posesCenter[i];
			Eigen::Vector2f imagePosClean = camera.projectFromWorld(xn);
			Eigen::Vector2f noise(error_distribution(mRandomDevice), error_distribution(mRandomDevice));
			Eigen::Vector2f imagePos = imagePosClean + noise;

			//Skip if outside of image
			if (imagePos[0] < 0 || imagePos[1] < 0 || imagePos[0] > camera.getImageSize()[0] || imagePos[1] > camera.getImageSize()[1])
				continue;

			//Save distortion and noise errors
			Eigen::Vector2f imagePosNoDistortion = camera.projectFromDistorted(xn.hnormalized());
			distortionError.push_back((imagePosClean - imagePosNoDistortion).norm());
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
		MYAPP_LOG << "Frame " << i << "\n";
		if (newFrame->getMeasurements().empty())
			MYAPP_LOG << "AAAAHHHH NO MEASUREMENTS!\n";
		Eigen::Map<Eigen::ArrayXf> _distortionError(distortionError.data(), distortionError.size());
		Eigen::Map<Eigen::ArrayXf> _noiseError(noiseError.data(), noiseError.size());
		MYAPP_LOG << "  Measurement count: " << newFrame->getMeasurements().size() << "\n";
		MYAPP_LOG << "  Max distortion error: " << _distortionError.maxCoeff() << "\n";
		MYAPP_LOG << "  Max noise error: " << _noiseError.maxCoeff() << "\n";

		//Get homography
		Eigen::Matrix<uchar, Eigen::Dynamic, 1> mask(refPoints.size());
		cv::Mat1b mask_cv(refPoints.size(), 1, mask.data());

		cv::Mat H;
		H = cv::findHomography(refPoints, imgPoints, cv::RANSAC, 3 * expectedPixelNoiseStd, mask_cv);
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
		cvH = hest.estimateCeres(cvH, imgPoints, refPoints, octaveVec, 3 * expectedPixelNoiseStd, inliersVec);

		//Set final
		newFrame->setPose(eutils::FromCV(cvH));

		//Add
		newMap->addKeyframe(std::move(newFrame));
	}

	return std::move(newMap);
}

}
