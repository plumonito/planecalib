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

	mSystem = app->getSystem();
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

	//Add features
	//mTrackerPose = &mTracker->getCurrentPose();
	for (auto &p : mSystem->getMap().getFeatures())
	{
		const Feature &feature = *p;
		Eigen::Vector4f color = StaticColors::Green();

		mImagePoints.push_back(feature.getPosition());
		mImagePointColors.push_back(color);
	}
	//for ()

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

	//Draw image points
	glPointSize(8);
	mShaders->getColor().drawVertices(GL_POINTS, mImagePoints.data(), mImagePointColors.data(), mImagePoints.size());
}

} /* namespace dtslam */
