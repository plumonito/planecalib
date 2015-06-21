#include "BouguetInterface.h"
#include "planecalib/Map.h"
#include "planecalib/CameraModel.h"
#include "planecalib/HomographyEstimation.h"
#include <opencv2/calib3d.hpp>
#include "planecalib/HomographyCalibration.h"
#include "planecalib/PnpEstimation.h"
#include "matio.h"

namespace planecalib
{

std::unique_ptr<Map> BouguetInterface::loadCalib(const std::string &filename)
{
	//Vars to read
	int imageCount;
	Eigen::Vector2f fc, cc, kc;
	Eigen::Vector2i imageSize;
	std::vector<Eigen::Matrix2Xf> imagePoints;
	std::vector<Eigen::Matrix3f> homographies;

	mat_t *matFile;
	matFile = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
	if (!matFile)
	{
		MYAPP_LOG << "Error opening mat file: " << filename << "\n";
		return NULL;
	}

	matvar_t *matVar;
	double *matData;

	//Read number of images
	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "n_ima");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	matData = static_cast<double*>(matVar->data);
	imageCount = (int)matData[0];
	MYAPP_LOG << "Reading calib info for " << imageCount << " images...";
	Mat_VarFree(matVar);


	//Read image width
	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "nx");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	matData = static_cast<double*>(matVar->data);
	imageSize[0] = (int)matData[0];
	Mat_VarFree(matVar);

	//Read image height
	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "ny");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	matData = static_cast<double*>(matVar->data);
	imageSize[1] = (int)matData[0];
	Mat_VarFree(matVar);

	//Read calib data
	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "fc");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	matData = static_cast<double*>(matVar->data);
	fc[0] = (float)matData[0];
	fc[1] = (float)matData[1];
	Mat_VarFree(matVar);

	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "cc");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	matData = static_cast<double*>(matVar->data);
	cc[0] = (float)matData[0];
	cc[1] = (float)matData[1];
	Mat_VarFree(matVar);

	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "kc");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	matData = static_cast<double*>(matVar->data);
	kc[0] = (float)matData[0];
	kc[1] = (float)matData[1];
	Mat_VarFree(matVar);

	//Read image data
	imagePoints.resize(imageCount);
	homographies.resize(imageCount);
	for (int i = 0; i < imageCount; i++)
	{
		//Image points
		{
			std::stringstream ss;
			ss << "x_" << (i + 1);
			Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
			matVar = Mat_VarRead(matFile, ss.str().c_str());
			if (!matVar)
				throw std::runtime_error("Variable not found in mat file.");
			assert(matVar->rank == 2);
			assert(matVar->dims[0] == 2);
			assert(matVar->data_type == MAT_T_DOUBLE);
			assert(matVar->class_type == MAT_C_DOUBLE);

			Eigen::Matrix2Xd points;
			points.resize(2, matVar->dims[1]);
			memcpy(points.data(), matVar->data, matVar->nbytes);
			imagePoints[i] = points.cast<float>();
			Mat_VarFree(matVar);
		}

		//Homography
		{
			std::stringstream ss;
			ss << "H_" << (i + 1);
			Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
			matVar = Mat_VarRead(matFile, ss.str().c_str());
			if (!matVar)
				throw std::runtime_error("Variable not found in mat file.");
			assert(matVar->rank == 2);
			assert(matVar->dims[0] == 3 && matVar->dims[1] == 3);
			assert(matVar->data_type == MAT_T_DOUBLE);
			assert(matVar->class_type == MAT_C_DOUBLE);
			Eigen::Matrix3d h;
			memcpy(h.data(), matVar->data, matVar->nbytes);
			homographies[i] = h.cast<float>();
			Mat_VarFree(matVar);
		}
	}

	Mat_Close(matFile);

	//Create map
	int featureCount = imagePoints[0].cols();
	std::unique_ptr<Map> map(new Map);

	//Create features
	for (int i = 0; i < featureCount; i++)
	{
		std::unique_ptr<Feature> feature(new Feature);
		feature->setPosition(imagePoints[0].col(i));
		map->addFeature(std::move(feature));
	}

	//Create keyframes
	cv::Mat3b nullImg3(imageSize[1], imageSize[0]);
	cv::Mat1b nullImg1(imageSize[1], imageSize[0]);
	Eigen::Matrix<uchar, 1, 32> nullDescr;
	nullDescr.setZero();
	Eigen::Matrix3f refHinv = homographies[0].inverse();
	for (int k = 0; k< imageCount; k++)
	{
		std::unique_ptr<Keyframe> frame(new Keyframe);

		frame->init(nullImg3, nullImg1);
		frame->setPose(homographies[k] * refHinv);

		for (int i = 0; i < featureCount; i++)
		{
			std::unique_ptr<FeatureMeasurement> m(new FeatureMeasurement(map->getFeatures()[i].get(), frame.get(), imagePoints[k].col(i), 0, nullDescr.data()));
			frame->getMeasurements().push_back(m.get());
			map->getFeatures()[i]->getMeasurements().push_back(std::move(m));
		}

		map->addKeyframe(std::move(frame));
	}

	std::unique_ptr<CameraModel> camera( new CameraModel());
	//camera->init(fc[0], fc[1], cc[0], cc[1], imageSize[0], imageSize[1]);
	//camera->getDistortionModel().init(kc, camera->getMaxRadiusSq());
	throw std::logic_error("Not implemented");

	map->mCamera = std::move(camera);
	return std::move(map);
}

std::unique_ptr<Map> BouguetInterface::loadValidation(const CameraModel &camera, const std::string &filename)
{
	//Vars to read
	int imageCount;
	std::vector<Eigen::Matrix2Xf> imagePoints;
	std::vector<Eigen::Matrix3Xf> worldPoints;

	mat_t *matFile;
	matFile = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
	if (!matFile)
	{
		MYAPP_LOG << "Error opening mat file: " << filename << "\n";
		return NULL;
	}

	matvar_t *matVar;
	double *matData;

	//Read number of images
	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "n_ima");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	matData = static_cast<double*>(matVar->data);
	imageCount = (int)matData[0];
	MYAPP_LOG << "Reading valid info for " << imageCount << " images...";
	Mat_VarFree(matVar);


	//Read image data
	imagePoints.resize(imageCount);
	worldPoints.resize(imageCount);
	for (int i = 0; i < imageCount; i++)
	{
		//Image points
		{
			std::stringstream ss;
			ss << "x_" << (i + 1);
			Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
			matVar = Mat_VarRead(matFile, ss.str().c_str());
			if (!matVar)
				throw std::runtime_error("Variable not found in mat file.");
			assert(matVar->rank == 2);
			assert(matVar->dims[0] == 2);
			assert(matVar->data_type == MAT_T_DOUBLE);
			assert(matVar->class_type == MAT_C_DOUBLE);

			Eigen::Matrix2Xd points;
			points.resize(2, matVar->dims[1]);
			memcpy(points.data(), matVar->data, matVar->nbytes);
			imagePoints[i] = points.cast<float>();
			Mat_VarFree(matVar);
		}

		//World points
		{
			std::stringstream ss;
			ss << "X_" << (i + 1);
			Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
			matVar = Mat_VarRead(matFile, ss.str().c_str());
			if (!matVar)
				throw std::runtime_error("Variable not found in mat file.");
			assert(matVar->rank == 2);
			assert(matVar->dims[0] == 3);
			assert(matVar->data_type == MAT_T_DOUBLE);
			assert(matVar->class_type == MAT_C_DOUBLE);

			Eigen::Matrix3Xd points;
			points.resize(3, matVar->dims[1]);
			memcpy(points.data(), matVar->data, matVar->nbytes);
			worldPoints[i] = points.cast<float>();
			Mat_VarFree(matVar);
		}
	}

	Mat_Close(matFile);

	//Check that all world points are the same
	for (int i = 1; i < worldPoints.size(); i++)
	{
		if (!worldPoints[0].array().isApprox(worldPoints[i].array(), 0.0001))
			MYAPP_LOG << "ERROR: World points are not equal for all images!!!!\n";
	}

	//Create map
	int featureCount = imagePoints[0].cols();
	std::unique_ptr<Map> map(new Map);

	//Create features
	for (int i = 0; i < featureCount; i++)
	{
		std::unique_ptr<Feature> feature(new Feature);
		feature->setPosition(imagePoints[0].col(i));
		feature->mGroundTruthPosition3D = worldPoints[0].col(i);
		feature->mPosition3D = feature->mGroundTruthPosition3D;
		map->addFeature(std::move(feature));
	}

	//Translate to vectors
	std::vector<Eigen::Vector3f> worldPointsVec;
	worldPointsVec.resize(featureCount);
	for (int i = 0; i < featureCount; i++)
	{
		worldPointsVec[i] = worldPoints[0].col(i);
	}
	std::vector<std::vector<Eigen::Vector2f>> imagePointsVec;
	imagePointsVec.resize(imageCount);
	for (int k = 0; k < imageCount; k++)
	{
		imagePointsVec[k].resize(featureCount);
		for (int i = 0; i < featureCount; i++)
		{
			imagePointsVec[k][i] = imagePoints[k].col(i);
		}
	}

	//Create keyframes
	std::vector<float> scales(worldPointsVec.size(), 1);
	cv::Mat3b nullImg3(camera.getImageSize()[1], camera.getImageSize()[0]);
	cv::Mat1b nullImg1(camera.getImageSize()[1], camera.getImageSize()[0]);
	Eigen::Matrix<uchar, 1, 32> nullDescr;
	nullDescr.setZero();
	for (int k = 0; k< imageCount; k++)
	{
		std::unique_ptr<Keyframe> frame(new Keyframe);

		frame->init(nullImg3, nullImg1);

		//Find pose
		PnPRansac ransac;
		ransac.setParams(3, 10, 100, (int)(0.9f*featureCount));
		ransac.setData(&worldPointsVec, &imagePointsVec[k], &scales, &camera);
		ransac.doRansac();
		frame->mPose3DR = ransac.getBestModel().first.cast<float>();
		frame->mPose3DT = ransac.getBestModel().second.cast<float>();
		//frame->setPose(homographies[k] * refHinv);

		for (int i = 0; i < featureCount; i++)
		{
			std::unique_ptr<FeatureMeasurement> m(new FeatureMeasurement(map->getFeatures()[i].get(), frame.get(), imagePointsVec[k][i], 0, nullDescr.data()));
			frame->getMeasurements().push_back(m.get());
			map->getFeatures()[i]->getMeasurements().push_back(std::move(m));
		}

		map->addKeyframe(std::move(frame));
	}

	return std::move(map);
}

}
