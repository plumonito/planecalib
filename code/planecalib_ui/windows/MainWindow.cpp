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
#include "../tests/BouguetInterface.h"
#include "planecalib/PnpEstimation.h"

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
	mKeyBindings.addBinding(false, 'v', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::loadValidationData), "Load validation data.");
	mKeyBindings.addBinding(false, 'b', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::doFullBA), "Full BA.");
	mKeyBindings.addBinding(false, 'h', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::doHomographyBA), "Homography BA.");
	mKeyBindings.addBinding(false, 't', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTest), "Synthetic test.");
	mKeyBindings.addBinding(false, 'y', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTestCompareUsingGroundTruth), "Synthetic test. Compare the metric BA of self-calib and calib.");
	mKeyBindings.addBinding(false, 'u', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTestNormalAngle), "Synthetic test for sensitivity to normal angle.");
	mKeyBindings.addBinding(false, 'i', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTestNormalizationWithNoise), "Synthetic test for sensitivity to normalization.");
	mKeyBindings.addBinding(false, 'I', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::synthTestNormalizationWithFrames), "Synthetic test for sensitivity to normalization.");

	mRefTexture.create(GL_LUMINANCE, eutils::ToSize(imageSize));
	mRefTexture.update(mSystem->getMap().getKeyframes()[0]->getImage(0));
	mDisplayTexture.create(GL_LUMINANCE, eutils::ToSize(imageSize));

	return true;
}

void MainWindow::updateState()
{
	shared_lock<shared_mutex> lockRead(mSystem->getMap().getMutex());

	mTracker = &mSystem->getTracker();
	mMap = &mSystem->getMap();
	const TrackingFrame *trackingFrame = mTracker->getFrame();

	//Check display frame idx
	if (mDisplayFrameIdx < -1)
		mDisplayFrameIdx = mMap->getKeyframes().size() - 1;
	else if (mDisplayFrameIdx >= mMap->getKeyframes().size())
		mDisplayFrameIdx = -1;


	//Clear all
	mDisplayPoints.clear();
	mDisplayColors.clear();
	mImagePoints.clear();
	mImagePointColors.clear();
	mImageLines.clear();
	mImageLineColors.clear();
	mFrameHomographies.clear();
	mFrameColors.clear();

	//Tracker pose
	mIsLost = mTracker->isLost();
	const Eigen::Matrix3fr T = eutils::GetTranslateHomography(-mTracker->getCamera().getPrincipalPoint());
	const Eigen::Matrix3fr Ti = eutils::GetTranslateHomography(mTracker->getCamera().getPrincipalPoint());
	mTrackerPose = Ti*mTracker->getCurrentPose2D()*T;

	//Display points for small thumbnail
	const Keyframe *displayFrame;
	Eigen::Vector4f colors[] = { StaticColors::Blue(), StaticColors::Green(), StaticColors::Red() };
	if (mDisplayFrameIdx == -1)
	{
		displayFrame = NULL;
		//Use this to show matches with warped image
		//mDisplayTextureTarget = mDisplayTexture.getTarget();
		//mDisplayTextureId = mDisplayTexture.getId();

		//if (trackingFrame && trackingFrame->getWarpedPyramid().getOctaveCount())
		//{
		//	mDisplayTexture.update(trackingFrame->getWarpedPyramid()[0]);
		//	for (auto &m : trackingFrame->getMatches())
		//	{
		//		mDisplayPoints.push_back(eutils::FromCV(m.getKeypoint().pt));
		//		mDisplayColors.push_back(colors[m.getOctave()]);
		//	}
		//}

		// Use this to show keypoints on original image
		mDisplayTextureTarget = mCurrentImageTextureTarget;
		mDisplayTextureId = mCurrentImageTextureId;
		if (trackingFrame && trackingFrame->getWarpedPyramid().getOctaveCount())
		{
			int maxOctave = std::min(3, mTracker->getFrame()->getWarpedPyramid().getOctaveCount());
			for (int octave = 0; octave < maxOctave; octave++)
			{
				for (auto &kp : mTracker->getFrame()->getWarpedKeypoints(octave))
				{
					mDisplayPoints.push_back(eutils::FromCV(kp.pt));
					mDisplayColors.push_back(colors[octave]);
				}
			}
		}

	}
	else
	{
		displayFrame = mMap->getKeyframes()[mDisplayFrameIdx].get();
		mDisplayTexture.update(displayFrame->getImage(0));
		mDisplayTextureTarget = mDisplayTexture.getTarget();
		mDisplayTextureId = mDisplayTexture.getId();

		for (auto &mPtr : displayFrame->getMeasurements())
		{
			auto &m = *mPtr;
			mDisplayPoints.push_back(m.getPosition());
			mDisplayColors.push_back(colors[m.getOctave()]);
		}
	}

	//Add features
	//mTrackerPose = &mTracker->getCurrentPose();
	for (auto &p : mMap->getFeatures())
	{
		const Feature &feature = *p;

		Eigen::Vector4f color;
		const FeatureMatch *match=NULL;
		if (trackingFrame)
			match = trackingFrame->getMatch(&feature);
		if (match)
		{
			color = StaticColors::Blue();
			//Eigen::Vector3f xn = eutils::HomographyPoint(mTracker->getCurrentPose2D(), match->getPosition()).homogeneous();
			//Eigen::Vector2f pos = mSystem->getCamera().projectFromWorld(xn);
			//Add line
			//mImageLines.push_back(feature.getPosition());
			//mImageLines.push_back(pos);
			//mImageLineColors.push_back(StaticColors::Yellow());
			//mImageLineColors.push_back(StaticColors::Blue());

			Eigen::Vector2f posRef = mSystem->getCamera().projectFromScaleSpace(feature.getPosition());
			mImagePoints.push_back(posRef);
			mImagePointColors.push_back(StaticColors::Green(0.7f));
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

		const Eigen::Matrix3fr T = eutils::GetTranslateHomography(-mSystem->getCamera().getPrincipalPoint());
		const Eigen::Matrix3fr Ti = eutils::GetTranslateHomography(mSystem->getCamera().getPrincipalPoint());

		auto H = (Ti*frame.getPose()*T).inverse().eval();

		mFrameHomographies.push_back(H);
		mFrameColors.push_back(color);
	}

	Eigen::Vector4f color;
	mFrameHomographies.push_back(mTrackerPose.inverse());
	if (mSystem->getTracker().isLost())
		color = StaticColors::Red();
	else if (mSystem->mKeyframeAdded)
		color = StaticColors::Green();
	else if (mDisplayFrameIdx == -1)
		color = StaticColors::Blue();
	else
		color = StaticColors::White();
	mFrameColors.push_back(color);
}

void MainWindow::resize()
{
	auto screenSize = UserInterfaceInfo::Instance().getScreenSize();
	float imgAspect = (float)mImageSize[0] / mImageSize[1];
	int smallImgWidth = (int)(screenSize[0] * 0.2f);
	int smallImgHeight = (int)(smallImgWidth / imgAspect);

	mTiler.configDevice(Eigen::Vector2i::Zero(), screenSize, 1, 1);

	mTiler.addTile(0);
	mTiler.setImageMVP(0, 2 * mImageSize);
	Eigen::Matrix4f offsetMat;
	offsetMat << 1, 0, 0, (float)mImageSize[0] / 2, 0, 1, 0, (float)mImageSize[1] / 2, 0, 0, 1, 0, 0, 0, 0, 1;
	mTiler.multiplyMVP(0, offsetMat);

	Eigen::Vector2i smallImgSize(smallImgWidth, smallImgHeight);
	Eigen::Vector2i smallImgOrigin(screenSize[0] - smallImgSize[0], 0);
	mTiler.addAbsoluteTile(smallImgOrigin, smallImgSize);
	mTiler.setImageMVP(1, mImageSize);
}

void MainWindow::draw()
{
	glPointSize(3.0f);

	mTiler.setActiveTile(1);

	mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTexture().renderTexture(mDisplayTextureTarget, mDisplayTextureId, mImageSize, 1.0f);
			
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().drawVertices(GL_POINTS, mDisplayPoints.data(), mDisplayColors.data(), mDisplayPoints.size());


	mTiler.setActiveTile(0);
	mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTextureWarp().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	//Draw textures
	mShaders->getTexture().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), mImageSize, 1.0f);
	if (!mIsLost)
		mShaders->getTextureWarp().renderTextureFixedAlpha(mCurrentImageTextureTarget, mCurrentImageTextureId, mTrackerPose, 0.5f, mImageSize);
	
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
	mTiler.setFullScreen();
	auto screenSize = UserInterfaceInfo::Instance().getScreenSize();
	auto mvp = ViewportTiler::GetImageSpaceMvp(UserInterfaceInfo::Instance().getScreenAspect(), screenSize);
	mShaders->getText().setMVPMatrix(mvp);
	mShaders->getText().setActiveFontSmall();
	mShaders->getText().setRenderCharHeight(PlaneCalibApp::kDefaultFontHeight);
	mShaders->getText().setCaret(Eigen::Vector2f(screenSize[0] - 300, 30));
	mShaders->getText().setColor(StaticColors::Green());
	{
		TextRendererStream ts(mShaders->getText());
		ts << std::fixed << std::setprecision(2) << std::setw(6);
		ts << "Closed-form Fx=" << mSystem->getCalib().getInitialAlpha() << "\n";
		ts << "\nRefined:\n";

		ts << std::fixed << std::setprecision(3) << std::setw(4);
		ts << " Normal=[" << mSystem->getNormal().transpose() << "]\n";

		ts << std::fixed << std::setprecision(2) << std::setw(6);
		ts << " F  = " << mSystem->getCamera().getFocalLength().transpose() << "\n";
		ts << " PP  = " << mSystem->getCamera().getPrincipalPoint().transpose() << "\n";

		ts << std::fixed << std::setprecision(3) << std::setw(4);
		ts << " Distortion = [" << mSystem->getCamera().getDistortionModel().getParams().transpose() << "]";
	}
}

void MainWindow::loadBouguetCalib()
{
	const std::string kDefaultFilename("Calib_results.mat");
	std::string filename;
	std::cout << "Load Bouguet calib data\n" << "Enter filename ([ ]='" << kDefaultFilename << "'): ";;
	std::cin >> filename;
	if (filename.empty() || filename.size() == 1)
		filename = kDefaultFilename;

	BouguetInterface b;
	auto map = b.loadCalib(filename);
	mSystem->setCamera(*map->mCamera);
	mSystem->setMap(std::move(map));
	updateState();
}

void MainWindow::loadValidationData()
{
	std::string answer;

	//Read camera
	std::cout << "Read camera? ([y,n]): ";
	std::cin >> answer;
	if (answer.size() == 1 && answer[0] == 'y')
	{
		float fx, fy, u0, v0, width, height, lambda;
		std::cout << "fx=";
		std::cin >> fx;
		std::cout << "fy=";
		std::cin >> fy;
		std::cout << "u0=";
		std::cin >> u0;
		std::cout << "v0=";
		std::cin >> v0;
		std::cout << "lambda=";
		std::cin >> lambda;
		std::cout << "width=";
		std::cin >> width;
		std::cout << "height=";
		std::cin >> height;

		CameraModel camera;
		camera.init(Eigen::Vector2f(u0,v0), Eigen::Vector2f(fx, fy), Eigen::Vector2i(width, height));
		camera.getDistortionModel().init(lambda);
		mSystem->setCamera(camera);
	}

	//Mat filename
	const std::string kDefaultFilename("calib_data.mat");
	std::string filename;
	std::cout << "Load validation data\n" << "Enter filename ([ ]='" << kDefaultFilename << "'): ";
	std::cin >> answer;
	if (answer.empty() || answer.size() == 1)
		filename = kDefaultFilename;
	else
		filename = answer;


	BouguetInterface b;
	auto map = b.loadValidation(mSystem->getCamera(), filename);
	mSystem->setMap(std::move(map));
	mSystem->doValidationBA();
	updateState();
}
void MainWindow::doHomographyBA()
{
	mSystem->doHomographyBA();
	mSystem->doHomographyCalib(true);
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
		errorFocal = (exp.getFocalLength() - ref.getFocalLength()).norm();
		errorP0 = (exp.getPrincipalPoint() - ref.getPrincipalPoint()).norm();
		errorDist0 = exp.getDistortionModel().getLambda() - ref.getDistortionModel().getLambda();
	}

	float errorFocal;
	float errorP0;
	float errorDist0;
};

void MainWindow::synthTest()
{
	CameraModel camera;
	camera.init(Eigen::Vector2f(320, 240), Eigen::Vector2f(600, 600), Eigen::Vector2i(640, 480));
	camera.getDistortionModel().init(0.1);

	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.setCamera(&camera);
	generator.mVerbose = true;

	CalibrationError error;

	MatlabDataLog::AddValue("errorKeyName", "'noiseStd'");

	for (noiseStd = 0; noiseStd <= 10; noiseStd += 0.5)
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
			mSystem->doHomographyCalib(true);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);

			//Only BA
			mSystem->setUse3DGroundTruth(true);
			mSystem->setFix3DPoints(false);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalBA", error.errorFocal);
			MatlabDataLog::AddValue("errorP0BA", error.errorP0);
			MatlabDataLog::AddValue("errorDist0BA", error.errorDist0);

			//Only BA fixed
			mSystem->setUse3DGroundTruth(true);
			mSystem->setFix3DPoints(true);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalBAFixed", error.errorFocal);
			MatlabDataLog::AddValue("errorP0BAFixed", error.errorP0);
			MatlabDataLog::AddValue("errorDist0BAFixed", error.errorDist0);

			MatlabDataLog::Stream().flush();
		}
	}

	mImageSize = camera.getImageSize();
	updateState();
}

void MainWindow::synthTestNormalAngle()
{
	CameraModel camera;
	camera.init(Eigen::Vector2f(320, 240), Eigen::Vector2f(600, 600), Eigen::Vector2i(640, 480));
	camera.getDistortionModel().init(0.1);

	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.mVerbose = true;
	generator.setCamera(&camera);
	generator.setNoiseStd(noiseStd);

	mSystem->setUse3DGroundTruth(false);
	mSystem->setFix3DPoints(false);

	CalibrationError error;

	MatlabDataLog::AddValue("errorKeyName", "'normalAngle'");

	float normalAngle = 10 * M_PI / 180;
	for (normalAngle = 0; normalAngle < 45 * M_PI / 180; normalAngle += 1 * M_PI / 180)
	{
		int kk = 0;
		for (kk = 0; kk < 300; kk++)
		{
			MYAPP_LOG << "-------------Synth test, normalAngle=" << (normalAngle * 180 / M_PI) << ", iter=" << kk << "----------------\n";
			std::unique_ptr<Map> map = generator.generateVariableNormal(normalAngle);

			//Calib
			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));
			mSystem->doHomographyBA();
			mSystem->doHomographyCalib(true);
			mSystem->doFullBA();

			//Record noise
			MatlabDataLog::AddValue("errorKey", normalAngle);
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);

			MatlabDataLog::Stream().flush();
		}
	}

	mImageSize = camera.getImageSize();
	updateState();
}

void MainWindow::synthTestNormalizationWithNoise()
{
	CameraModel camera;
	camera.init(Eigen::Vector2f(320, 240), Eigen::Vector2f(600, 600), Eigen::Vector2i(640, 480));
	camera.getDistortionModel().init(0.1);

	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.mVerbose = true;
	generator.setCamera(&camera);
	generator.setNoiseStd(noiseStd);

	mSystem->setUse3DGroundTruth(false);
	mSystem->setFix3DPoints(false);

	CalibrationError error;

	MatlabDataLog::AddValue("errorKeyName", "'noiseStd'");

	float normalAngle = 20 * M_PI / 180;
	for (noiseStd = 0; noiseStd <= 10; noiseStd += 0.5)
		//for (normalAngle = 0; normalAngle < 45 * M_PI / 180; normalAngle += 1 * M_PI / 180)
	{
		int kk = 0;
		for (kk = 0; kk < 300; kk++)
		{
			MYAPP_LOG << "-------------Synth test for normalization, noiseStd=" << noiseStd << ", iter=" << kk << "----------------\n";
			generator.setNoiseStd(noiseStd);
			std::unique_ptr<Map> map = generator.generateSyntheticMap(normalAngle);

			MatlabDataLog::AddValue("errorKey", noiseStd);

			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));
			mSystem->doHomographyBA();

			//Calib
			mSystem->setUseNormalizedConstraints(false);
			mSystem->doHomographyCalib(false);
			//mSystem->doFullBA();

			//Record noise
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalNoNorm", error.errorFocal);
			MatlabDataLog::AddValue("errorP0NoNorm", error.errorP0);
			MatlabDataLog::AddValue("errorDist0NoNorm", error.errorDist0);

			//Calib
			mSystem->setUseNormalizedConstraints(true);
			mSystem->doHomographyCalib(false);
			//mSystem->doFullBA();

			//Record noise
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);

			MatlabDataLog::Stream().flush();
		}
	}

	mImageSize = camera.getImageSize();
	updateState();
}

void MainWindow::synthTestNormalizationWithFrames()
{
	CameraModel camera;
	camera.init(Eigen::Vector2f(320, 240), Eigen::Vector2f(600, 600), Eigen::Vector2i(640, 480));
	camera.getDistortionModel().init(0.1);

	float noiseStd = 3 / 3;

	SceneGenerator generator;
	generator.mVerbose = true;
	generator.setCamera(&camera);
	generator.setNoiseStd(noiseStd);

	mSystem->setUse3DGroundTruth(false);
	mSystem->setFix3DPoints(false);

	CalibrationError error;

	MatlabDataLog::AddValue("errorKeyName", "'frameCount'");

	float normalAngle = 20 * M_PI / 180;
	for (int frameCount = 0; frameCount <= 30; frameCount++)
	{
		int kk = 0;
		for (kk = 0; kk < 300; kk++)
		{
			MYAPP_LOG << "-------------Synth test for normalization, frameCount=" << frameCount << ", noiseStd=" << noiseStd << ", iter=" << kk << "----------------\n";
			std::unique_ptr<Map> map = generator.generateRandomMap(frameCount, normalAngle);

			MatlabDataLog::AddValue("errorKey", frameCount);

			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));
			mSystem->doHomographyBA();

			//Calib
			mSystem->setUseNormalizedConstraints(false);
			mSystem->doHomographyCalib(false);

			//Record noise
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalNoNorm", error.errorFocal);
			MatlabDataLog::AddValue("errorP0NoNorm", error.errorP0);
			MatlabDataLog::AddValue("errorDist0NoNorm", error.errorDist0);

			//Calib
			mSystem->setUseNormalizedConstraints(true);
			mSystem->doHomographyCalib(false);

			//Record noise
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);


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
	camera.init(Eigen::Vector2f(320, 240), Eigen::Vector2f(600, 600), Eigen::Vector2i(640, 480));
	camera.getDistortionModel().init(0.1);

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
			MYAPP_LOG << "-------------Synth test, compare all, frameCount=" << frameCount << "iter=" << kk << "----------------\n";
			std::unique_ptr<Map> map = generator.generateRandomMap(frameCount);

			//Record key
			MatlabDataLog::AddValue("errorKey", frameCount);

			//Prepare scene
			mSystem->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
			mSystem->setMap(std::move(map));

			//Only BA
			mSystem->setUse3DGroundTruth(false);
			mSystem->setFix3DPoints(false);
			mSystem->doHomographyBA();
			mSystem->doHomographyCalib(true);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocal", error.errorFocal);
			MatlabDataLog::AddValue("errorP0", error.errorP0);
			MatlabDataLog::AddValue("errorDist0", error.errorDist0);

			//Only BA
			mSystem->setUse3DGroundTruth(true);
			mSystem->setFix3DPoints(false);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalBA", error.errorFocal);
			MatlabDataLog::AddValue("errorP0BA", error.errorP0);
			MatlabDataLog::AddValue("errorDist0BA", error.errorDist0);

			//Only BA fixed
			mSystem->setUse3DGroundTruth(true);
			mSystem->setFix3DPoints(true);
			mSystem->doFullBA();

			//Record 
			error.compute(camera, mSystem->getCamera());
			MatlabDataLog::AddValue("errorFocalBAFixed", error.errorFocal);
			MatlabDataLog::AddValue("errorP0BAFixed", error.errorP0);
			MatlabDataLog::AddValue("errorDist0BAFixed", error.errorDist0);

			MatlabDataLog::Stream().flush();
		}
	}

	mImageSize = camera.getImageSize();
	updateState();
}

void MainWindow::synthTest2()
{
	CameraModel camera;
	camera.init(Eigen::Vector2f(320, 240), Eigen::Vector2f(600, 600), Eigen::Vector2i(640, 480));
	camera.getDistortionModel().init(0.1);
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
			mSystem->doHomographyCalib(true);
			mSystem->doFullBA();

			//Record noise
			noiseStdVec.push_back(noiseStd);
			CalibrationError error;
			error.compute(camera, mSystem->getCamera());
			errorFocal.push_back(error.errorFocal);
			errorP0.push_back(error.errorP0);
			errorDist0.push_back(error.errorDist0);
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