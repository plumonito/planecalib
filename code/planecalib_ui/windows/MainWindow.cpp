/*
 * ARWindow.cpp
 *
 *  Created on: 7.4.2014
 *      Author: dan
 */

#include "MainWindow.h"
#include "planecalib/PlaneCalibSystem.h"
#include "planecalib/Map.h"
#include "planecalib/PoseTracker.h"
#include "planecalib/HomographyCalibration.h"
#include "../PlaneCalibApp.h"
#include "../tests/SceneGenerator.h"
#include "matio.h"

namespace planecalib
{
void MainWindow::showHelp() const
{
	BaseWindow::showHelp();
	MYAPP_LOG << "Shows an overlay of the triangulated features and the cube."
		"Three display modes: show matches, show reprojected features with 3 measurements, and show all reprojected features.\n";
}

bool MainWindow::init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize)
{
	BaseWindow::init(app, imageSize);

	mSystem = &app->getSystem();

	resize();

	//mKeyBindings.addBinding(false, 't', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&ARWindow::toggleDisplayType), "Toggle display mode.");
	mKeyBindings.addBinding(false, '+', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::nextDisplayFrame), "Show details on next frame.");
	mKeyBindings.addBinding(false, '-', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::prevDisplayFrame), "Show details on prev frame.");
	mKeyBindings.addBinding(false, 'l', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::loadBouguetCalib), "Load bouguet calibration data.");
	mKeyBindings.addBinding(false, 'b', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::doFullBA), "Full BA.");
	mKeyBindings.addBinding(false, 'h', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::doHomographyBA), "Homography BA.");
	mKeyBindings.addBinding(false, 't', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTest), "Synthetic test.");
	mKeyBindings.addBinding(false, 'y', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTestCompareUsingGroundTruth), "Synthetic test. Compare the metric BA of self-calib and calib.");
	mKeyBindings.addBinding(false, 'u', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTestNormalAngle), "Synthetic test for sensitivity to normal angle.");
	mKeyBindings.addBinding(false, 'i', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTestNormalization), "Synthetic test for sensitivity to normalization.");

	mRefTexture.create(GL_RGB, eutils::ToSize(imageSize));
	mRefTexture.update(mSystem->getMap().getKeyframes()[0]->getColorImage());
	mDisplayTexture.create(GL_RGB, eutils::ToSize(imageSize));

	return true;
}

void MainWindow::updateState()
{
	shared_lock<shared_mutex> lockRead(mSystem->getMap().getMutex());
	
	mTracker = &mSystem->getTracker();
	mMap = &mSystem->getMap();

	//Display frame
	Keyframe *displayFrame;
	if (mDisplayFrameIdx < -1)
		mDisplayFrameIdx = mMap->getKeyframes().size() - 1;
	else if (mDisplayFrameIdx >= mMap->getKeyframes().size())
		mDisplayFrameIdx = -1;

	mDisplayPoints.clear();

	if (mDisplayFrameIdx == -1)
		displayFrame = NULL;
	else
	{
		displayFrame = mMap->getKeyframes()[mDisplayFrameIdx].get();
		mDisplayTexture.update(displayFrame->getColorImage());

		for (auto &mPtr : displayFrame->getMeasurements())
		{
			auto &m = *mPtr;
			mDisplayPoints.push_back(m.getPosition());
		}
	}

	//Clear all
	mImagePoints.clear();
	mImagePointColors.clear();
	mImageLines.clear();
	mImageLineColors.clear();
	mFrameHomographies.clear();
	mFrameColors.clear();

	mTrackerPose = mTracker->getCurrentPose();
	mIsLost = mTracker->isLost();

	//Add features
	//mTrackerPose = &mTracker->getCurrentPose();
	for (auto &p : mMap->getFeatures())
	{
		const Feature &feature = *p;
		
		Eigen::Vector4f color;
		auto match = mTracker->getMatch(&feature);
		if (match)
		{
			color = StaticColors::Blue();

			//Add line
			mImageLines.push_back(feature.getPosition());
			mImageLines.push_back(match->getPosition());
			mImageLineColors.push_back(StaticColors::Yellow());
			mImageLineColors.push_back(StaticColors::Blue());
		}
		else
			color = StaticColors::Green(0.7f);

		//mImagePoints.push_back(feature.getPosition());
		//mImagePointColors.push_back(color);
	}
	//auto &refPoints = mSystem->getTracker().refPoints;
	//auto &imgPoints = mSystem->getTracker().imgPoints;
	//for (int i = 0; i < refPoints.size(); i++)
	//{
	//	mImageLines.push_back(eutils::FromCV(refPoints[i]));
	//	mImageLines.push_back(eutils::FromCV(imgPoints[i]));
	//	mImageLineColors.push_back(StaticColors::Yellow());
	//	mImageLineColors.push_back(StaticColors::Blue());
	//}

	for (auto &framep : mMap->getKeyframes())
	{
		auto &frame = *framep;
		Eigen::Vector4f color;
		if (&frame == displayFrame)
			color = StaticColors::Blue();
		else
			color = StaticColors::Gray();

		mFrameHomographies.push_back(frame.getPose().inverse());
		mFrameColors.push_back(color);
	}

	Eigen::Vector4f color;
	mFrameHomographies.push_back(mSystem->getTracker().getCurrentPose().inverse());
	if (mSystem->getTracker().isLost())
		color = StaticColors::Red();
	else
		color = StaticColors::White();
	mFrameColors.push_back(color);
}

void MainWindow::resize()
{
    mTiler.configDevice(Eigen::Vector2i::Zero(),UserInterfaceInfo::Instance().getScreenSize(),3,3);

	mTiler.addTile(0, 0, -1.0f, 3, 2);
	mTiler.setImageMVP(0, 2*mImageSize);
	Eigen::Matrix4f offsetMat;
	offsetMat << 1, 0, 0, (float)mImageSize[0] / 2, 0, 1, 0, (float)mImageSize[1] / 2, 0, 0, 1, 0, 0, 0, 0, 1;
	mTiler.multiplyMVP(0, offsetMat);

	mTiler.addTile(0, 2);
	mTiler.setImageMVP(1, mImageSize);
	mTiler.addTile(1, 2);
	mTiler.setImageMVP(2, mImageSize);

	mTiler.addTile(2, 2);
	mTiler.setImageMVP(3, Eigen::Vector2i(100,100));
}

void MainWindow::draw()
{
	glPointSize(3.0f);

	if (mDisplayFrameIdx != -1)
	{
		mTiler.setActiveTile(1);
		mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
		mShaders->getTexture().renderTexture(mDisplayTexture.getTarget(), mDisplayTexture.getId(), mImageSize, 1.0f);
		
		mShaders->getColor().setMVPMatrix(mTiler.getMVP());
		mShaders->getColor().drawVertices(GL_POINTS, mDisplayPoints.data(), mDisplayPoints.size(), StaticColors::Green());
	}

	mTiler.setActiveTile(2);
	mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTexture().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mImageSize, 1.0f);
	if (mTracker->getFrame())
	{
		std::vector<Eigen::Vector2f> points;
		for (auto &kp : mTracker->getFrame()->getKeypoints(0))
		{
			points.push_back(eutils::FromCV(kp.pt));
		}
		mShaders->getColor().setMVPMatrix(mTiler.getMVP());
		mShaders->getColor().drawVertices(GL_POINTS, points.data(), points.size(), StaticColors::Blue());
	}

	mTiler.setActiveTile(0);
    mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTextureWarp().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	//Draw textures
	mShaders->getTexture().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), mImageSize, 1.0f);
	if (!mIsLost)
		mShaders->getTextureWarp().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mTrackerPose, 0.5f, mImageSize);

	std::vector<Eigen::Vector2f> pointsSrc;
	pointsSrc.push_back(Eigen::Vector2f(0, 0));
	pointsSrc.push_back(Eigen::Vector2f(mImageSize[0], 0));
	pointsSrc.push_back(Eigen::Vector2f(mImageSize[0], mImageSize[1]));
	pointsSrc.push_back(Eigen::Vector2f(0, mImageSize[1]));

	mShaders->getWarpPos().setMVPMatrix(mTiler.getMVP());

	for (int i = 0; i < mFrameHomographies.size(); i++)
	{
		mShaders->getWarpPos().setHomography(mFrameHomographies[i]);
		mShaders->getWarpPos().drawVertices(GL_LINE_LOOP, &pointsSrc[0], pointsSrc.size(), mFrameColors[i]);
	}

	//Draw image lines
	mShaders->getColor().drawVertices(GL_LINES, mImageLines.data(), mImageLineColors.data(), mImageLines.size());

	//Draw image points
	glPointSize(8);
	mShaders->getColor().drawVertices(GL_POINTS, mImagePoints.data(), mImagePointColors.data(), mImagePoints.size());

	//K
	mTiler.setActiveTile(3);
	mShaders->getText().setMVPMatrix(mTiler.getMVP());
	mShaders->getText().setActiveFontSmall();
	mShaders->getText().setRenderCharHeight(4);
	mShaders->getText().setCaret(Eigen::Vector2f(0, 0));
	mShaders->getText().setColor(StaticColors::Green());
	{
		TextRendererStream ts(mShaders->getText());
		ts << "Alpha0=" << mSystem->getCalib().getInitialAlpha() << "\n";
		ts << "Normal=" << mSystem->getNormal() << "\nK=" << mSystem->getCamera().getK() << "\nDistortion=" << mSystem->getCamera().getDistortionModel().getCoefficients().transpose();
	}
}

void MainWindow::loadBouguetCalib()
{
	//Vars to read
	int imageCount;
	Eigen::Vector2i imageSize;
	std::vector<Eigen::Matrix2Xf> imagePoints;
	std::vector<Eigen::Matrix3f> homographies;

	std::string filename("Calib_results.mat");

	mat_t *matFile;
	matFile = Mat_Open(filename.c_str(), MAT_ACC_RDONLY);
	if (!matFile)
	{
		MYAPP_LOG << "Error opening mat file: " << filename << "\n";
		return;
	}

	matvar_t *matVar;

	//Read number of images
	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile,"n_ima");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	imageCount = (int)(*static_cast<double*>(matVar->data));
	MYAPP_LOG << "Reading calib info for " << imageCount << " images...";
	Mat_VarFree(matVar);


	//Read image width
	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "nx");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	imageSize[0] = (int)(*static_cast<double*>(matVar->data));
	Mat_VarFree(matVar);

	//Read image height
	Mat_Rewind(matFile); //Hack: must call or Mat_VarRead might fail
	matVar = Mat_VarRead(matFile, "ny");
	if (!matVar)
		throw std::runtime_error("Variable not found in mat file.");
	imageSize[1] = (int)(*static_cast<double*>(matVar->data));
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
	cv::Mat3b nullImg3(imageSize[1],imageSize[0]);
	cv::Mat1b nullImg1(imageSize[1], imageSize[0]);
	Eigen::Matrix<uchar, 1, 32> nullDescr;
	nullDescr.setZero();
	Eigen::Matrix3f refHinv = homographies[0].inverse();
	for (int k = 0; k< imageCount; k++)
	{
		std::unique_ptr<Keyframe> frame(new Keyframe);
		
		frame->init(nullImg3, nullImg1);
		frame->setPose(homographies[k]*refHinv);

		for (int i = 0; i < featureCount; i++)
		{
			std::unique_ptr<FeatureMeasurement> m(new FeatureMeasurement(map->getFeatures()[i].get(), frame.get(), imagePoints[k].col(i), 0, nullDescr.data()));
			frame->getMeasurements().push_back(m.get());
			map->getFeatures()[i]->getMeasurements().push_back(std::move(m));
		}

		map->addKeyframe(std::move(frame));
	}

	mSystem->setMap(std::move(map));
	updateState();
}

void MainWindow::doHomographyBA()
{
	mSystem->doHomographyBA();
}

void MainWindow::doFullBA()
{
	mSystem->doFullBA();
}

class CalibrationError
{
public:
	void compute(const CameraModel &ref, const CameraModel &exp)
	{
		errorFocal = Eigen::Vector2f(exp.getFx() - ref.getFx(), exp.getFy() - ref.getFy()).norm();
		errorP0 = Eigen::Vector2f(exp.getU0() - ref.getU0(), exp.getV0() - ref.getV0()).norm();
		errorDist0 = exp.getDistortionModel().getCoefficients()[0] - ref.getDistortionModel().getCoefficients()[0];
		errorDist1 = exp.getDistortionModel().getCoefficients()[1] - ref.getDistortionModel().getCoefficients()[1];
	}

	float errorFocal;
	float errorP0;
	float errorDist0;
	float errorDist1;
};

void MainWindow::synthTest()
{
	CameraModel camera;
	camera.init(600, 600, 320, 240, 640, 480);
	camera.getDistortionModel().init(Eigen::Vector2f(0.1, -0.01), camera.getMaxRadiusSq());

	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.setCamera(&camera);

	CalibrationError error;

	MatlabDataLog::AddValue("errorKeyName", "'noiseStd'");

	for (noiseStd = 0; noiseStd <= 10; noiseStd+=0.5)
	{
		int kk = 0;
		for (kk = 0; kk < 300; kk++)
		{
			MYAPP_LOG << "-------------Synth test, compare all, noise=" << noiseStd << ", iter=" << kk << "----------------\n";
			generator.setNoiseStd(noiseStd);
			std::unique_ptr<Map> map = generator.generateSyntheticMap();

			//Record key
			MatlabDataLog::AddValue("errorKey", noiseStd);

			//Prepare scene
			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));

			//Only BA
			mSystem->setUse3DGroundTruth(false);
			mSystem->setFix3DPoints(false);
			mSystem->doHomographyBA();
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1", error.errorDist1);

			//Only BA
			mSystem->setUse3DGroundTruth(true);
			mSystem->setFix3DPoints(false);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalBA", error.errorFocal);
			MatlabDataLog::AddValue("errorP0BA", error.errorP0);
			MatlabDataLog::AddValue("errorDist0BA", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1BA", error.errorDist1);

			//Only BA fixed
			mSystem->setUse3DGroundTruth(true);
			mSystem->setFix3DPoints(true);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalBAFixed", error.errorFocal);
			MatlabDataLog::AddValue("errorP0BAFixed", error.errorP0);
			MatlabDataLog::AddValue("errorDist0BAFixed", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1BAFixed", error.errorDist1);

			MatlabDataLog::Stream().flush();
		}
	}

	mImageSize = camera.getImageSize();
	updateState();
}

void MainWindow::synthTestNormalAngle()
{
	CameraModel camera;
	camera.init(600, 600, 320, 240, 640, 480);
	camera.getDistortionModel().init(Eigen::Vector2f(0.1, -0.01), camera.getMaxRadiusSq());
	//camera.getDistortionModel().init(Eigen::Vector2f(0.0, 0.0), camera.getMaxRadiusSq());

	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.mVerbose = true;
	generator.setCamera(&camera);
	generator.setNoiseStd(noiseStd);

	mSystem->setUse3DGroundTruth(false);
	mSystem->setFix3DPoints(false);

	CalibrationError error;

	MatlabDataLog::AddValue("errorKeyName", "'normalAngle'");

	float normalAngle = 10*M_PI/180;
	for (normalAngle = 0; normalAngle < 45 * M_PI / 180; normalAngle+=1*M_PI/180)
	{
		int kk = 0;
		for (kk = 0; kk < 300; kk++)
		{
			MYAPP_LOG << "-------------Synth test, normalAngle=" << (normalAngle*180/M_PI) << ", iter=" << kk << "----------------\n";
			std::unique_ptr<Map> map = generator.generateVariableNormal(normalAngle);

			//Calib
			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));
			mSystem->doHomographyBA();
			mSystem->doFullBA();

			//Record noise
			MatlabDataLog::AddValue("errorKey", normalAngle);
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1", error.errorDist1);

			MatlabDataLog::Stream().flush();
		}
	}

	mImageSize = camera.getImageSize();
	updateState();
}

void MainWindow::synthTestNormalization()
{
	CameraModel camera;
	camera.init(600, 600, 320, 240, 640, 480);
	camera.getDistortionModel().init(Eigen::Vector2f(0.1, -0.01), camera.getMaxRadiusSq());
	//camera.getDistortionModel().init(Eigen::Vector2f(0.0, 0.0), camera.getMaxRadiusSq());

	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.mVerbose = true;
	generator.setCamera(&camera);
	generator.setNoiseStd(noiseStd);

	mSystem->setUse3DGroundTruth(false);
	mSystem->setFix3DPoints(false);

	CalibrationError error;

	MatlabDataLog::AddValue("errorKeyName", "'normalAngle'");

	float normalAngle = 0 * M_PI / 180;
	for (normalAngle = 0; normalAngle < 45 * M_PI / 180; normalAngle += 1 * M_PI / 180)
	{
		int kk = 0;
		for (kk = 0; kk < 300; kk++)
		{
			MYAPP_LOG << "-------------Synth test for normalization, normalAngle=" << (normalAngle * 180 / M_PI) << ", iter=" << kk << "----------------\n";
			std::unique_ptr<Map> map = generator.generateVariableNormal(normalAngle);

			MatlabDataLog::AddValue("errorKey", normalAngle);

			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));

			//Calib
			mSystem->setUseNormalizedConstraints(true);
			mSystem->doHomographyBA();
			//mSystem->doFullBA();

			//Record noise
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1", error.errorDist1);

			//Calib
			mSystem->setUseNormalizedConstraints(false);
			mSystem->doHomographyBA();
			//mSystem->doFullBA();

			//Record noise
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalNoNorm", error.errorFocal);
			MatlabDataLog::AddValue("errorP0NoNorm", error.errorP0);
			MatlabDataLog::AddValue("errorDist0NoNorm", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1NoNorm", error.errorDist1);

			MatlabDataLog::Stream().flush();
		}
	}

	mImageSize = camera.getImageSize();
	updateState();
}

void MainWindow::storeSceneToMat(const Map &map)
{

}

void MainWindow::synthTestCompareUsingGroundTruth()
{
	CameraModel camera;
	camera.init(600, 600, 320, 240, 640, 480);
	camera.getDistortionModel().init(Eigen::Vector2f(0.1, -0.01), camera.getMaxRadiusSq());

	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.setCamera(&camera);
	generator.setNoiseStd(noiseStd);

	CalibrationError error;

	MatlabDataLog::AddValue("errorKeyName", "'frameCount'");

	int frameCount = 50;
	for (frameCount = 3; frameCount <= 50; frameCount++)
	{
		int kk = 0;
		for (kk = 0; kk < 300; kk++)
		{
			MYAPP_LOG << "-------------Synth test, compare all, iter=" << kk << "----------------\n";
			std::unique_ptr<Map> map = generator.generateRandomPoses(frameCount);

			//Record key
			MatlabDataLog::AddValue("errorKey", frameCount);

			//Prepare scene
			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));

			//Only BA
			mSystem->setUse3DGroundTruth(false);
			mSystem->setFix3DPoints(false);
			mSystem->doHomographyBA();
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1", error.errorDist1);

			//Only BA
			mSystem->setUse3DGroundTruth(true);
			mSystem->setFix3DPoints(false);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalBA", error.errorFocal);
			MatlabDataLog::AddValue("errorP0BA", error.errorP0);
			MatlabDataLog::AddValue("errorDist0BA", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1BA", error.errorDist1);

			//Only BA fixed
			mSystem->setUse3DGroundTruth(true);
			mSystem->setFix3DPoints(true);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalBAFixed", error.errorFocal);
			MatlabDataLog::AddValue("errorP0BAFixed", error.errorP0);
			MatlabDataLog::AddValue("errorDist0BAFixed", error.errorDist0);
			MatlabDataLog::AddValue("errorDist1BAFixed", error.errorDist1);

			MatlabDataLog::Stream().flush();
		}
	}

	mImageSize = camera.getImageSize();
	updateState();
}

void MainWindow::synthTest2()
{
	CameraModel camera;
	camera.init(600, 600, 320, 240, 640, 480);
	camera.getDistortionModel().init(Eigen::Vector2f(0.1, -0.01), camera.getMaxRadiusSq());
	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.setCamera(&camera);

	std::unique_ptr<Map> newMap;
	//newMap = generator.generateSyntheticMap(camera, noiseStd);
	//newMap = generator.generateRandomPoses(camera,10);
	//mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
	//mSystem->setMap(std::move(newMap));
	//mSystem->doHomographyBA();
	//mSystem->doFullBA();
	return;

	//size_t varDims[2];
	//mat_t *mat = Mat_Create("vars.mat",NULL);

	//std::vector<matvar_t*> varScenes;

	std::vector<float> noiseStdVec;
	std::vector<float> errorFocal;
	std::vector<float> errorP0;
	std::vector<float> errorDist0;
	std::vector<float> errorDist1;
	for (noiseStd = 0; noiseStd < 10; noiseStd += 0.5)
	{
		for (int kk = 0; kk < 300; kk++)
		{
			MYAPP_LOG << "-------------Synth test, noiseStd=" << noiseStd << ", iter=" << kk << "----------------\n";
			generator.setNoiseStd(noiseStd);
			std::unique_ptr<Map> map = generator.generateSyntheticMap();

			////Store scene
			//std::vector<matvar_t*> varFrames;
			//for (auto &framePtr : map->getKeyframes())
			//{
			//	auto &frame = *framePtr;
			//	Eigen::MatrixXd imgPos, worldPos;
			//	imgPos.resize(2, frame.getMeasurements().size());
			//	worldPos.resize(3, frame.getMeasurements().size());

			//	for (int i = 0; i < imgPos.cols(); i++)
			//	{
			//		auto &m = *frame.getMeasurements()[i];
			//		imgPos.col(i) = m.getPosition().cast<double>();
			//		worldPos.col(i) = m.getFeature().mPosition3D.cast<double>();
			//	}

			//	Eigen::Matrix3d R = frame.mPose3DR.cast<double>();
			//	Eigen::Vector3d T = frame.mPose3DT.cast<double>();

			//	matvar_t *varsFrame[5];
			//	varDims[0] = 2; varDims[1] = imgPos.cols();
			//	varsFrame[0] = Mat_VarCreate("imgPos", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, varDims, imgPos.data(), 0);
			//	varDims[0] = 3; varDims[1] = worldPos.cols();
			//	varsFrame[1] = Mat_VarCreate("worldPos", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, varDims, worldPos.data(), 0);
			//	varDims[0] = 3; varDims[1] = 3;
			//	varsFrame[2] = Mat_VarCreate("R", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, varDims, R.data(), 0);
			//	varDims[0] = 3; varDims[1] = 1;
			//	varsFrame[3] = Mat_VarCreate("t", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, varDims, T.data(), 0);
			//	varsFrame[4] = NULL;

			//	varDims[0] = 1; varDims[1] = 1;
			//	varFrames.push_back(Mat_VarCreate("frame", MAT_C_STRUCT, MAT_T_STRUCT, 2, varDims, varsFrame, 0));
			//}
			//varDims[0] = varFrames.size(); varDims[1] = 1;
			//varFrames.push_back(NULL);
			//varScenes.push_back(Mat_VarCreate("frames", MAT_C_CELL, MAT_T_CELL, 2, varDims, varFrames.data(), 0));

			//Calib
			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));
			mSystem->doHomographyBA();
			mSystem->doFullBA();

			//Record noise
			noiseStdVec.push_back(noiseStd);
			CalibrationError error;
			error.compute(camera, mSystem->getCamera());
			errorFocal.push_back(error.errorFocal);
			errorP0.push_back(error.errorP0);
			errorDist0.push_back(error.errorDist0);
			errorDist1.push_back(error.errorDist1);
		}
	}
	//varDims[0] = varScenes.size(); varDims[1] = 1;
	//varScenes.push_back(NULL);
	//matvar_t *varRoot = Mat_VarCreate("scenes", MAT_C_CELL, MAT_T_CELL, 2, varDims, varScenes.data(), 0);
	//Mat_VarWrite(mat, varRoot, MAT_COMPRESSION_NONE);
	//Mat_VarFree(varRoot);
	//Mat_Close(mat);

	MatlabDataLog::Instance().AddValue("noise", noiseStdVec);
	MatlabDataLog::Instance().AddValue("errorFocal", errorFocal);
	MatlabDataLog::Instance().AddValue("errorP0", errorP0);
	MatlabDataLog::Instance().AddValue("errorDist0", errorDist0);
	MatlabDataLog::Instance().AddValue("errorDist1", errorDist1);

	updateState();
}

} 
