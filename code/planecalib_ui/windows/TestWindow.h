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
class ErrorClass;

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

	void *mRefKeyframe;
	cv::Mat1b mRefImg;
	cv::Mat1s mRefDx;
	cv::Mat1s mRefDy;

	std::vector<Eigen::Vector3i> mEvalPositions;
	cv::Mat3b mEvalPosImg;

	Eigen::Matrix3fr mPose0;
	Eigen::Matrix3fr mPoseAffine;
	Eigen::Matrix3fr mPoseHomography;

	//Draw data
	TextureHelper mRefTexture;
	TextureHelper mImgTexture;
	TextureHelper mEvalPositionsTexture;
	TextureHelper mCostAffineTexture;
	TextureHelper mCostHomographyTexture;

	std::vector<Eigen::Vector2f> mCornerPosAffine;
	std::vector<Eigen::Vector2f> mCornerPosHomography;

	void loadRefFrame();
	void alignAffine(const cv::Mat1b &img, const cv::Mat1s &imgDx, const cv::Mat1s &imgDy, Eigen::Matrix3fr &pose, TextureHelper &tex);
	void alignHomography(const cv::Mat1b &img, const cv::Mat1s &imgDx, const cv::Mat1s &imgDy, Eigen::Matrix3fr &pose, TextureHelper &tex);
};

} 

#endif /* ARWINDOW_H_ */
