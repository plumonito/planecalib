/*
 * ARWindow.h
 *
 *  Created on: 7.4.2014
 *      Author: dan
 */

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include "BaseWindow.h"
#include <Eigen/Dense>

namespace planecalib
{

class PlaneCalibSystem;
class Map;
class FrameTrackingData;
class PoseTracker;

class MainWindow: public BaseWindow
{
public:
	MainWindow():
		BaseWindow("MainWindow")
	{}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize);
	void showHelp() const;

	void updateState();
    void resize();

//    void touchDown(int id, int x, int y);
//    void touchMove(int x, int y);
//    void touchUp(int id, int x, int y);

    void draw();

	void doFullBA();
	void logData();

protected:
    ViewportTiler mTiler;

	PlaneCalibSystem *mSystem;
	Map *mMap;
    PoseTracker *mTracker;

	bool mIsLost;
	Eigen::Matrix3fr mTrackerPose;

	TextureHelper mRefTexture;

	//Draw data
	std::vector<Eigen::Vector2f> mImagePoints;
	std::vector<Eigen::Vector4f> mImagePointColors;

	std::vector<Eigen::Vector2f> mImageLines;
	std::vector<Eigen::Vector4f> mImageLineColors;
};

} /* namespace dtslam */

#endif /* ARWINDOW_H_ */
