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
	mTracker = &mSystem->getTracker();
	mMap = &mSystem->getMap();

	resize();

	//mKeyBindings.addBinding(false, 't', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&ARWindow::toggleDisplayType), "Toggle display mode.");
	mKeyBindings.addBinding(false, 'l', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MainWindow::logData), "Log data.");

	mRefTexture.create(GL_RGB, eutils::ToSize(imageSize));
	mRefTexture.update(mSystem->getMap().getKeyframes()[0]->getColorImage());

	return true;
}

void MainWindow::updateState()
{
	shared_lock<shared_mutex> lockRead(mSystem->getMap().getMutex());

	//Clear all
	mImagePoints.clear();
	mImagePointColors.clear();
	mImageLines.clear();
	mImageLineColors.clear();

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

	const float pointAlpha = 0.6f;
}

void MainWindow::resize()
{
    mTiler.configDevice(Eigen::Vector2i::Zero(),UserInterfaceInfo::Instance().getScreenSize(),3,3);

	mTiler.addTile(0, 0, -1.0f, 3, 2);
	mTiler.setImageMVP(0, mImageSize);

	mTiler.addTile(0, 2);
	mTiler.setImageMVP(1, mImageSize);
	mTiler.addTile(1, 2);
	mTiler.setImageMVP(2, mImageSize);

	mTiler.addTile(2, 2);
	mTiler.setImageMVP(3, Eigen::Vector2i(100,100));
}

void MainWindow::draw()
{
	mTiler.setActiveTile(1);
	mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTexture().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), mImageSize, 1.0f);

	mTiler.setActiveTile(2);
	mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTexture().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mImageSize, 1.0f);

	mTiler.setActiveTile(0);
    mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTextureWarp().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	//Draw textures
	mShaders->getTexture().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), mImageSize, 1.0f);
	if (!mIsLost)
		mShaders->getTextureWarp().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mTrackerPose, 0.5f, mImageSize);

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
		ts << "K=" << mSystem->getCalib().getK() << "\nNormal=" << mSystem->getCalib().getNormal();
	}
}

void MainWindow::logData()
{
	MatlabDataLog::Instance().SetValue("K", mSystem->getCalib().getK());
	MatlabDataLog::Instance().SetValue("n", mSystem->getCalib().getNormal());

	//Add features
	std::unordered_map<Feature *, int> featureMap;
	int id=0;
	for each (auto &feature in mSystem->getMap().getFeatures())
	{
		featureMap.insert(std::make_pair(feature.get(), ++id));
		MatlabDataLog::Instance().AddValue("features", feature->getPosition());
	}

	//Add keyframes
	for each (auto &frame in mSystem->getMap().getKeyframes())
	{
		MatlabDataLog::Instance().AddCell("H", frame->getPose());
		MatlabDataLog::Instance().AddCell("H", frame->getPose());
	}
}

} /* namespace dtslam */
