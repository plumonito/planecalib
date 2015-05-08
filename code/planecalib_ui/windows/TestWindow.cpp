
#include "TestWindow.h"
#include "planecalib/PlaneCalibSystem.h"
#include "planecalib/HomographyCalibration.h"
#include "planecalib/Map.h"
#include "planecalib/PoseTracker.h"
#include "../PlaneCalibApp.h"
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace planecalib
{
void TestWindow::showHelp() const
{
	BaseWindow::showHelp();
	MYAPP_LOG << "Just tests.";
}

class ImageInterpolation : public ceres::SizedCostFunction<1, 2> {
public:
	ImageInterpolation(const cv::Mat1b &img, const cv::Mat1s &imgDx, const cv::Mat1s &imgDy);

	virtual bool Evaluate(double const* const* parameters,
		double* residuals,
		double** jacobians) const;

protected:
	const cv::Mat1b &mImg;
	const cv::Mat1s &mImgDx;
	const cv::Mat1s &mImgDy;

	template<class T>
	T interpolate(const cv::Mat_<T> &img, const double x, const double y) const;
};

ImageInterpolation::ImageInterpolation(const cv::Mat1b &img, const cv::Mat1s &imgDx, const cv::Mat1s &imgDy):
	mImg(img), mImgDx(imgDx), mImgDy(imgDy)
{
}

bool ImageInterpolation::Evaluate(double const* const* parameters,
	double* residuals,
	double** jacobians) const
{
	//int u0 = (int)parameters[0][0];
	//int v0 = (int)parameters[0][1];
	//residuals[0] = mImg(v0, u0);

	double u = parameters[0][0];
	double v = parameters[0][1];
	residuals[0] = interpolate(mImg, u, v);
	if (jacobians)
	{
		//jacobians[0][0] = mImgDx(v0, u0);
		//jacobians[0][1] = mImgDy(v0, u0);
		jacobians[0][0] = interpolate(mImgDx, u, v);
		jacobians[0][1] = interpolate(mImgDy, u, v);
	}
	return true;
}

template<class T>
T ImageInterpolation::interpolate(const cv::Mat_<T> &img, const double x, const double y) const
{
	int x0 = (int)x;
	int y0 = (int)y;
	
	double dx = x - x0;
	double dy = y - y0;
	
	int x1 = x0+1;
	if (x1 == img.cols)
		x1--;
	int y1 = y0+1;
	if (y1 == img.rows)
		y1--;

	int v00 = (int)img(y0, x0);
	int v10 = (int)img(y1, x0);
	double v0 = v00*(1 - dy) + v10*dy;

	int v01 = (int)img(y0, x1);
	int v11 = (int)img(y1, x1);
	double v1 = v01*(1 - dy) + v11*dy;

	T val = T(v0*(1-dx)+v1*dx);
	return val;
}


class ErrorClass
{
public:
	ErrorClass(const cv::Mat1b &refImg, const cv::Mat1s &refDx, const cv::Mat1s &refDy, const std::vector<Eigen::Vector3i> &evalPositions, const cv::Mat1b &img, const cv::Mat1s &imgDx, const cv::Mat1s &imgDy);

	template<class T>
	bool operator() (const T * const paramsA, const T * const paramsB, const T * const paramsC, T *residuals)  const;

protected:
	const cv::Mat1b &mRefImg;
	const cv::Mat1s &mRefDx;
	const cv::Mat1s &mRefDy;
	const std::vector<Eigen::Vector3i> &mEvalPositions;
	const cv::Mat1b &mImg;
	const cv::Mat1s &mImgDx;
	const cv::Mat1s &mImgDy;

	ceres::CostFunctionToFunctor<1, 2> mInterp;
};

ErrorClass::ErrorClass(const cv::Mat1b &refImg, const cv::Mat1s &refDx, const cv::Mat1s &refDy, const std::vector<Eigen::Vector3i> &evalPositions, const cv::Mat1b &img, const cv::Mat1s &imgDx, const cv::Mat1s &imgDy) :
mRefImg(refImg), mRefDx(refDx), mRefDy(refDy), mEvalPositions(evalPositions), mImg(img), mImgDx(imgDx), mImgDy(imgDy), mInterp(new ImageInterpolation(img, imgDx, imgDy))
{
}

template<class T>
bool ErrorClass::operator() (const T * const paramsA, const T * const paramsB, const T * const paramsC, T *residuals)  const
{
	Eigen::Map<Eigen::Matrix3Xi> evalPositionsMap((int*)mEvalPositions.data(), 3, mEvalPositions.size());
	Eigen::Matrix<T, 3, Eigen::Dynamic> refPos = evalPositionsMap.cast<T>();

	Eigen::Matrix<T, 3, 3> H;
	//H << T(1), T(0), params[0], T(0), T(1), params[1], T(0), T(0), T(1);
	H << paramsB[0], paramsB[1], paramsA[0], paramsB[2], paramsB[3], paramsA[1], paramsC[0], paramsC[1], paramsC[2];

	Eigen::Matrix<T, 3, Eigen::Dynamic> imgPos3 = H*refPos;

	Eigen::Matrix<T, 2, Eigen::Dynamic> imgPos;
	//Eigen::Matrix<T, 2, 100> imgPos;
	imgPos.resize(2, imgPos3.cols());
	imgPos.row(0) = imgPos3.row(0).array() / imgPos3.row(2).array();
	imgPos.row(1) = imgPos3.row(1).array() / imgPos3.row(2).array();

	int validCount = 0;
	for (int i = 0; i < imgPos.cols(); i++)
	{
		T *imgPosi = &imgPos(0, i);

		double imgUd = CeresUtils::ToDouble(imgPosi[0]);
		double imgVd = CeresUtils::ToDouble(imgPosi[1]);
		if (imgUd < 0 || imgVd < 0 || imgUd > mImg.cols - 1 || imgVd > mImg.rows - 1)
		{
			residuals[i] = T(0);
			continue;
		}

		T imgVal;
		mInterp(imgPosi, &imgVal);

		auto &refPosi = mEvalPositions[i];
		T refVal = T(mRefImg(refPosi[1], refPosi[0]));

		residuals[i] = imgVal - refVal;
		validCount++;
	}

	T validCountT(validCount);
	for (int i = 0; i < imgPos.cols(); i++)
	{
		residuals[i] /= validCountT;
	}

	return true;
}

bool TestWindow::init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize)
{
	BaseWindow::init(app, imageSize);

	const int kMinGradientSq = 10 * 10;

	mRefImg = mApp->getSystem().getMap().getKeyframes()[0]->getImage(0);
	
	cv::Sobel(mRefImg, mRefDx, CV_16S, 1, 0, 1);
	cv::Sobel(mRefImg, mRefDy, CV_16S, 0, 1, 1);

	mEvalPosImg.create(mRefImg.size());

	for (Eigen::Vector3i p(0, 0, 1); p[1] < mRefImg.rows; p[1]++)
	{
		for (p[0] = 0; p[0] < mRefImg.cols; p[0]++)
		{
			const auto &dx = mRefDx(p[1], p[0]);
			const auto &dy = mRefDy(p[1], p[0]);
			const auto dmSq = dx*dx + dy*dy;
			if (dmSq > kMinGradientSq)
			{
				mEvalPositions.push_back(p);
				mEvalPosImg(p[1], p[0]) = cv::Vec3b(0,255,0);
			}
			else
				mEvalPosImg(p[1], p[0]) = cv::Vec3b(0, 0, 0);
		}
	}

	mPose = Eigen::Matrix3f::Identity();

	mRefTexture.create(GL_LUMINANCE, mRefImg.size());
	mRefTexture.update(mRefImg);

	mEvalPositionsTexture.create(GL_RGB, mEvalPosImg.size());
	mEvalPositionsTexture.update(mEvalPosImg);

	resize();

	return true;
}

void TestWindow::updateState()
{
	shared_lock<shared_mutex> lockRead(mApp->getSystem().getMap().getMutex());

	const Keyframe *frame = mApp->getSystem().getTracker().getFrame();
	if (!frame)
		return;

	cv::Mat1b img;
	cv::Mat1s imgDx;
	cv::Mat1s imgDy;

	img = frame->getImage(0);
	cv::Sobel(img, imgDx, CV_16S, 1, 0, 1);
	cv::Sobel(img, imgDy, CV_16S, 0, 1, 1);

	//Solver options
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::CGNR;
	//options.dense_linear_algebra_library_type = ceres::LAPACK;

	options.max_num_iterations = 500;
	options.num_threads = 4;
	options.num_linear_solver_threads = 4;
	options.logging_type = ceres::SILENT;
	options.minimizer_progress_to_stdout = false;

	Eigen::Vector2d paramsA;
	Eigen::Vector4d paramsB;
	Eigen::Vector3d paramsC;
	//paramsA << 0,0;
	//paramsB << 1, 0, 0, 1;
	//paramsC << 0, 0, 1;
	paramsA << mPose(0, 2), mPose(1, 2);
	paramsB << mPose(0, 0), mPose(0, 1), mPose(1, 0), mPose(1, 1);
	paramsC << mPose(2, 0), mPose(2, 1), mPose(2, 2);

	//Problem
	ceres::Problem problem;
	problem.AddParameterBlock(paramsA.data(), paramsA.size());
	problem.AddParameterBlock(paramsB.data(), paramsB.size());
	problem.AddParameterBlock(paramsC.data(), paramsC.size());
	//problem.SetParameterBlockConstant(paramsB.data());
	//problem.SetParameterBlockConstant(paramsC.data());
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorClass, ceres::DYNAMIC, 2, 4, 3>(
		new ErrorClass(mRefImg, mRefDx,mRefDy,mEvalPositions,img,imgDx,imgDy), mEvalPositions.size()), 
		NULL, paramsA.data(), paramsB.data(), paramsC.data());

	ceres::Solver::Summary summary;
	{
		ProfileSection s("TestSolve");
		ceres::Solve(options, &problem, &summary);
	}

	MYAPP_LOG << summary.FullReport();
	MYAPP_LOG << "ParamsA: " << paramsA << "\n";
	MYAPP_LOG << "ParamsB: " << paramsB << "\n";
	MYAPP_LOG << "ParamsC: " << paramsC << "\n";
	mPose << paramsB[0], paramsB[1], paramsA[0], paramsB[2], paramsB[3], paramsA[1], paramsC[0], paramsC[1], paramsC[2];
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
	mShaders->getTextureWarp().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	//Draw texture
	mShaders->getTextureWarp().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mPose, mImageSize);
	mShaders->getTexture().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), eutils::FromSize(mRefTexture.getSize()), 0.7f);
	//mShaders->getTexture().renderTexture(mEvalPositionsTexture.getTarget(), mEvalPositionsTexture.getId(), eutils::FromSize(mEvalPositionsTexture.getSize()), 0.3f);
}

} /* namespace dtslam */
