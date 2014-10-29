/*
 * BaseWindow.cpp
 *
 *  Created on: 28.2.2014
 *      Author: dan
 */

#include "BaseWindow.h"
#include <GL/freeglut.h>
#include <planecalib/log.h>
#include "../PlaneCalibApp.h"

namespace planecalib
{

bool BaseWindow::init(PlaneCalibApp *app, const cv::Size &imageSize)
{
	assert(app);
	assert(imageSize.width>0 && imageSize.height>0);

	mIsInitialized = true;
	mApp = app;
	mShaders = &app->getShaders();
	mImageSize = imageSize;

	mKeyBindings.clear();

	return true;
}

void BaseWindow::showHelp() const
{
	MYAPP_LOG << "\n--" << mName << " help--\n";
	mKeyBindings.showHelp();
}

void BaseWindow::keyDown(bool isSpecial, unsigned char key)
{
	mKeyBindings.dispatchKeyDown(isSpecial, key);
}

void BaseWindow::keyUp(bool isSpecial, unsigned char key)
{
	mKeyBindings.dispatchKeyUp(isSpecial, key);
}

} /* namespace planecalib */
