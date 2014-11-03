/*
 * ARWindow.cpp
 *
 *  Created on: 7.4.2014
 *      Author: dan
 */

#include "MainWindow.h"
#include "planecalib/PlaneCalibSystem.h"
#include "planecalib/Map.h"
//#include "planecalib/PoseTracker.h"
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
	//mTracker = &app->getSystem().getTracker();

	resize();

	//mKeyBindings.addBinding(false, 't', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&ARWindow::toggleDisplayType), "Toggle display mode.");

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

	//Add features
	//mTrackerPose = &mTracker->getCurrentPose();
	for (auto &p : mSystem->getMap().getFeatures())
	{
		const Feature &feature = *p;
		
		Eigen::Vector4f color;
		auto match = mSystem->getTracker().getMatch(&feature);
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

		mImagePoints.push_back(feature.getPosition());
		mImagePointColors.push_back(color);
	}
	auto &refPoints = mSystem->getTracker().refPoints;
	auto &imgPoints = mSystem->getTracker().imgPoints;
	for (int i = 0; i < refPoints.size(); i++)
	{
		mImageLines.push_back(eutils::FromCV(refPoints[i]));
		mImageLines.push_back(eutils::FromCV(imgPoints[i]));
		mImageLineColors.push_back(StaticColors::Yellow());
		mImageLineColors.push_back(StaticColors::Blue());
	}

	const float pointAlpha = 0.6f;
}

void MainWindow::resize()
{
    mTiler.configDevice(Eigen::Vector2i::Zero(),UserInterfaceInfo::Instance().getScreenSize(),1);
	mTiler.fillTiles();
	mTiler.setImageMVP(0, mImageSize);
}

void MainWindow::draw()
{
    mTiler.setActiveTile(0);
    mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	//Draw textures
	mShaders->getTexture().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), mImageSize, 1.0f);
	mShaders->getTexture().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mImageSize, 0.5f);

	//Draw image lines
	mShaders->getColor().drawVertices(GL_LINES, mImageLines.data(), mImageLineColors.data(), mImageLines.size());

	//Draw image points
	glPointSize(8);
	mShaders->getColor().drawVertices(GL_POINTS, mImagePoints.data(), mImagePointColors.data(), mImagePoints.size());
}

} /* namespace dtslam */
