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

bool MainWindow::init(PlaneCalibApp *app, const cv::Size &imageSize)
{
	BaseWindow::init(app, imageSize);

	mSystem = app->getSystem();
	//mTracker = &app->getSystem().getTracker();

	resize();

	//mKeyBindings.addBinding(false, 't', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&ARWindow::toggleDisplayType), "Toggle display mode.");

	return true;
}

void MainWindow::updateState()
{
	//shared_lock<shared_mutex> lockRead(mSystem->getMap().getMutex());

	//Clear all
	mImagePoints.clear();
	mImagePointColors.clear();

	//Add features
	//mTrackerPose = &mTracker->getCurrentPose();

	const float pointAlpha = 0.6f;
}

void MainWindow::resize()
{
    mTiler.configDevice(cv::Rect2i(cv::Point2i(0,0),UserInterfaceInfo::Instance().getScreenSize()),1);
	mTiler.fillTiles();
	mTiler.setImageMVP(0, mImageSize);
}

void MainWindow::draw()
{
    mTiler.setActiveTile(0);
    mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	//Draw texture
	mShaders->getTexture().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mImageSize, 1.0f);

	//Draw image points
	glPointSize(8);
	//mShaders->getColor().drawVertices(GL_POINTS, mImagePoints.data(), mImagePointColors.data(), mImagePoints.size());
}

} /* namespace dtslam */
