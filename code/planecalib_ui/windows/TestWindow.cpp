
#include "TestWindow.h"
#include "planecalib/PlaneCalibSystem.h"
#include "planecalib/HomographyCalibration.h"
#include "planecalib/Map.h"
#include "planecalib/PoseTracker.h"
#include "../PlaneCalibApp.h"
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>

namespace planecalib
{
void TestWindow::showHelp() const
{
	BaseWindow::showHelp();
	MYAPP_LOG << "Just tests.";
}

class ErrorClass
{
public:
	ErrorClass();

	template<class T>
	bool operator()(const T * const params, T *residuals);

protected:
	cv::Mat1b mRefImg;
	cv::Mat1s mRefDx;
	cv::Mat1s mRefDy;

	std::vector<Eigen::Vector2f> mEvalPositions;
};

ErrorClass::ErrorClass()
{
	
	//for (int y=0)
}

template<class T>
bool ErrorClass::operator() (const T * const params, T *residuals)
{

}

bool TestWindow::init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize)
{
	BaseWindow::init(app, imageSize);

	mRefImg = mApp->getSystem().getMap().getKeyframes()[0]->getImage(0);
	
	cv::Sobel(mRefImg, mRefDx, CV_16S, 1, 0, 1);
	cv::Sobel(mRefImg, mRefDy, CV_16S, 0, 1, 1);

	mPose = Eigen::Matrix3f::Identity();

	mRefTexture.create(GL_LUMINANCE, mRefImg.size());
	mRefTexture.update(mRefImg);

	resize();

	return true;
}

void TestWindow::updateState()
{
	shared_lock<shared_mutex> lockRead(mApp->getSystem().getMap().getMutex());
}

void TestWindow::resize()
{
	mTiler.configDevice(Eigen::Vector2i::Zero(), UserInterfaceInfo::Instance().getScreenSize(), 1);
	mTiler.fillTiles();
	mTiler.setImageMVP(0, mImageSize);
}

void TestWindow::draw()
{
    mTiler.setActiveTile(0);
    mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	//Draw texture
	mShaders->getTexture().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), eutils::FromSize(mRefTexture.getSize()), 1.0f);
	mShaders->getTexture().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mImageSize, 0.3f);
}

} /* namespace dtslam */
