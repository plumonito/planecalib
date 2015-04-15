#include "SceneGenerator.h"
#include "planecalib/Map.h"
#include "matio.h"
#include "planecalib/CameraModel.h"
#include "planecalib/HomographyEstimation.h"
#include <opencv2/calib3d.hpp>

namespace planecalib
{

std::unique_ptr<Map> SceneGenerator::generateSyntheticMap(const CameraModel &camera, float measurementNoiseStd)
{
	std::unique_ptr<Map> newMap(new Map());

	cv::Mat3b nullImg3(camera.getImageSize()[1], camera.getImageSize()[0]);
	cv::Mat1b nullImg1(camera.getImageSize()[1], camera.getImageSize()[0]);
	Eigen::Matrix<uchar, 1, 32> nullDescr;
	nullDescr.setZero();

	float expectedPixelNoiseStd = std::max(0.3f, measurementNoiseStd);

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

	//Frame poses
	std::vector<Eigen::Matrix3fr> posesR;
	std::vector<Eigen::Vector3f> posesCenter;

	posesR.push_back(Eigen::Matrix3fr::Identity());
	posesCenter.push_back(Eigen::Vector3f(0, 0, -1));

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
	std::normal_distribution<float> error_distribution(0, (measurementNoiseStd==0) ? std::numeric_limits<float>::epsilon() : measurementNoiseStd);
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
			Eigen::Vector3f xn = posesR[i] * feature.mPosition3D - posesR[i] * posesCenter[i];
			Eigen::Vector2f imagePosClean = camera.projectFromWorld(xn);
			Eigen::Vector2f noise(error_distribution(mRandomDevice), error_distribution(mRandomDevice));
			Eigen::Vector2f imagePos = imagePosClean + noise;

			//Skip if outside of image
			if (imagePos[0] < 0 || imagePos[1] < 0 || imagePos[0] > camera.getImageSize()[0] || imagePos[1] > camera.getImageSize()[1])
				continue;

			//Save distortion and noise errors
			Eigen::Vector2f imagePosNoDistortion = camera.getDistortionModel().undistortPoint(imagePosClean);
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
