#ifndef TESTWINDOW_H_
#define TESTWINDOW_H_

#include "planecalib/eutils.h"
#include "planecalib/CameraModel.h"
#include "BaseWindow.h"
#include "../TextureHelper.h"

namespace planecalib
{

class Map;
class PoseTracker;

class TestWindow : public BaseWindow
{
public:
	TestWindow() :
		BaseWindow("TestWindow")
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

protected:
    ViewportTiler mTiler;

	cv::Mat1b mRefImg;
	cv::Mat1s mRefDx;
	cv::Mat1s mRefDy;
	Eigen::Matrix3fr mPose;

	//Draw data
	TextureHelper mRefTexture;
};

} 

#endif /* ARWINDOW_H_ */
