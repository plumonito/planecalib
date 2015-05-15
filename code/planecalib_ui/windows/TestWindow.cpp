
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
	void evalRaw(const T * const paramsA, const T * const paramsB, const T * const paramsC, T *residuals, T*validCount)  const;

	template<class T>
	void makeImg(const T * const paramsA, const T * const paramsB, const T * const paramsC, cv::Mat1b &img)  const;

	template<class T>
	bool operator() (const T * const paramsA, const T * const paramsB, const T * const paramsC, T *residuals)  const;

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
	T validCountT;

	this->evalRaw(paramsA, paramsB, paramsC, residuals, &validCountT);
	for (int i = 0; i < (int)mEvalPositions.size(); i++)
	{
		residuals[i] /= validCountT;
	}

	return true;
}

template<class T>
void ErrorClass::evalRaw(const T * const paramsA, const T * const paramsB, const T * const paramsC, T *residuals, T*validCountT)  const
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
	for (int i = 0; i < (int)mEvalPositions.size(); i++)
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

	*validCountT = T(validCount);
}

template<class T>
void ErrorClass::makeImg(const T * const paramsA, const T * const paramsB, const T * const paramsC, cv::Mat1b &img)  const
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

	img.create(mRefImg.size());
	img.setTo(0);

	int validCount = 0;
	for (int i = 0; i < (int)mEvalPositions.size(); i++)
	{
		T *imgPosi = &imgPos(0, i);

		double imgUd = CeresUtils::ToDouble(imgPosi[0]);
		double imgVd = CeresUtils::ToDouble(imgPosi[1]);
		if (imgUd < 0 || imgVd < 0 || imgUd > mImg.cols - 1 || imgVd > mImg.rows - 1)
		{
			continue;
		}

		T imgVal;
		mInterp(imgPosi, &imgVal);

		auto &refPosi = mEvalPositions[i];
		img(refPosi[1], refPosi[0]) = (uchar)imgVal;
	}
}

bool TestWindow::init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize)
{
	BaseWindow::init(app, imageSize);

	mRefTexture.create(GL_LUMINANCE, eutils::ToSize(imageSize));
	mImgTexture.create(GL_LUMINANCE, eutils::ToSize(imageSize));
	mCostAffineTexture.create(GL_LUMINANCE, eutils::ToSize(imageSize));
	mCostHomographyTexture.create(GL_LUMINANCE, eutils::ToSize(imageSize));
	mEvalPositionsTexture.create(GL_RGB, eutils::ToSize(imageSize));

	loadRefFrame();

	resize();

	return true;
}

void TestWindow::loadRefFrame()
{
	const int kMinGradientSq = 10 * 10;

	mRefKeyframe = mApp->getSystem().getMap().getKeyframes()[0].get();
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
			//if (dmSq > kMinGradientSq)
			{
				mEvalPositions.push_back(p);
				mEvalPosImg(p[1], p[0]) = cv::Vec3b(0, 255, 0);
			}
			//else
			//	mEvalPosImg(p[1], p[0]) = cv::Vec3b(0, 0, 0);
		}
	}

	mPoseAffine = mPoseHomography = Eigen::Matrix3f::Identity();

	mRefTexture.update(mRefImg);

	mCostAffineTexture.update(mRefImg);
	mCostHomographyTexture.update(mRefImg);

	mEvalPositionsTexture.update(mEvalPosImg);
}
void TestWindow::alignAffine(const cv::Mat1b &img, const cv::Mat1s &imgDx, const cv::Mat1s &imgDy, Eigen::Matrix3fr &pose, TextureHelper &tex)
{
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
	
	Eigen::Matrix3fr pose0;
	//pose0 = Eigen::Matrix3f::Identity();
	pose0 = pose;
	paramsA << pose0(0, 2), pose0(1, 2);
	paramsB << pose0(0, 0), pose0(0, 1), pose0(1, 0), pose0(1, 1);
	paramsC << pose0(2, 0), pose0(2, 1), pose0(2, 2);

	//Problem
	ceres::Problem problem;
	problem.AddParameterBlock(paramsA.data(), paramsA.size());
	problem.AddParameterBlock(paramsB.data(), paramsB.size());
	problem.AddParameterBlock(paramsC.data(), paramsC.size());
	//problem.SetParameterBlockConstant(paramsB.data());
	problem.SetParameterBlockConstant(paramsC.data());

	auto *errorFunc = new ErrorClass(mRefImg, mRefDx, mRefDy, mEvalPositions, img, imgDx, imgDy);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorClass, ceres::DYNAMIC, 2, 4, 3>(
		errorFunc, mEvalPositions.size()),
		NULL, paramsA.data(), paramsB.data(), paramsC.data());

	ceres::Solver::Summary summary;
	{
		ProfileSection s("TestSolve");
		ceres::Solve(options, &problem, &summary);
	}

	MYAPP_LOG << "-----------------Align affine-----------------\n";
	MYAPP_LOG << summary.FullReport();
	MYAPP_LOG << "ParamsA: " << paramsA << "\n";
	MYAPP_LOG << "ParamsB: " << paramsB << "\n";
	MYAPP_LOG << "ParamsC: " << paramsC << "\n";

	pose << paramsB[0], paramsB[1], paramsA[0], paramsB[2], paramsB[3], paramsA[1], paramsC[0], paramsC[1], paramsC[2];

	//Cost
	std::vector<double> residuals;
	double validCount;
	residuals.resize(errorFunc->mEvalPositions.size());
	errorFunc->evalRaw(paramsA.data(), paramsB.data(), paramsC.data(), residuals.data(), &validCount);
	MYAPP_LOG << "Valid count: " << validCount << "\n";

	cv::Mat1b costImg;
	costImg.create(errorFunc->mImg.size());
	costImg.setTo(0);
	for (int i = 0; i < errorFunc->mEvalPositions.size(); i++)
	{
		auto &pos = errorFunc->mEvalPositions[i];
		uchar val = cv::saturate_cast<uchar>(std::abs(residuals[i]) * 50);
		costImg(pos[1], pos[0]) = val;
	}

	tex.update(costImg);
}

void TestWindow::alignHomography(const cv::Mat1b &img, const cv::Mat1s &imgDx, const cv::Mat1s &imgDy, Eigen::Matrix3fr &pose, TextureHelper &tex)
{
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
	Eigen::Matrix3fr pose0;
	//pose0 = Eigen::Matrix3f::Identity();
	pose0 = pose;
	paramsA << pose0(0, 2), pose0(1, 2);
	paramsB << pose0(0, 0), pose0(0, 1), pose0(1, 0), pose0(1, 1);
	paramsC << pose0(2, 0), pose0(2, 1), pose0(2, 2);

	//Problem
	ceres::Problem problem;
	problem.AddParameterBlock(paramsA.data(), paramsA.size());
	problem.AddParameterBlock(paramsB.data(), paramsB.size());
	
	std::vector<int> constantIdx;
	constantIdx.push_back(2);
	problem.AddParameterBlock(paramsC.data(), paramsC.size(), new ceres::SubsetParameterization(3,constantIdx));

	//problem.SetParameterBlockConstant(paramsB.data());
	//problem.SetParameterBlockConstant(paramsC.data());

	auto *errorFunc = new ErrorClass(mRefImg, mRefDx, mRefDy, mEvalPositions, img, imgDx, imgDy);
	problem.AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorClass, ceres::DYNAMIC, 2, 4, 3>(
		errorFunc, mEvalPositions.size()),
		NULL, paramsA.data(), paramsB.data(), paramsC.data());

	ceres::Solver::Summary summary;
	{
		ProfileSection s("TestSolve");
		ceres::Solve(options, &problem, &summary);
	}

	MYAPP_LOG << "-----------------Align homography-----------------\n";
	MYAPP_LOG << summary.FullReport();
	MYAPP_LOG << "ParamsA: " << paramsA << "\n";
	MYAPP_LOG << "ParamsB: " << paramsB << "\n";
	MYAPP_LOG << "ParamsC: " << paramsC << "\n";

	pose << paramsB[0], paramsB[1], paramsA[0], paramsB[2], paramsB[3], paramsA[1], paramsC[0], paramsC[1], paramsC[2];	

	//Cost
	std::vector<double> residuals;
	double validCount;
	residuals.resize(errorFunc->mEvalPositions.size());
	errorFunc->evalRaw(paramsA.data(), paramsB.data(), paramsC.data(), residuals.data(), &validCount);
	MYAPP_LOG << "Valid count: " << validCount << "\n";

	cv::Mat1b costImg;
	costImg.create(errorFunc->mImg.size());
	costImg.setTo(0);
	for (int i = 0; i < errorFunc->mEvalPositions.size(); i++)
	{
		auto &pos = errorFunc->mEvalPositions[i];
		uchar val = cv::saturate_cast<uchar>(std::abs(residuals[i]) * 50);
		costImg(pos[1], pos[0]) = val;
	}

	tex.update(costImg);
}

void TestWindow::updateState()
{
	shared_lock<shared_mutex> lockRead(mApp->getSystem().getMap().getMutex());

	if (mApp->getSystem().getMap().getKeyframes()[0].get() != mRefKeyframe)
		loadRefFrame();

	const Keyframe *frame = mApp->getSystem().getTracker().getFrame();
	if (!frame)
		return;

	//mPose = Eigen::Matrix3f::Identity();
	//mPose0 = mPose;

	cv::Mat1b img;
	cv::Mat1s imgDx;
	cv::Mat1s imgDy;

	img = frame->getImage(0);
	cv::Sobel(img, imgDx, CV_16S, 1, 0, 1);
	cv::Sobel(img, imgDy, CV_16S, 0, 1, 1);

	mImgTexture.update(img);

	//mPoseAffine = mPoseHomography = Eigen::Matrix3fr::Identity();

	alignAffine(img, imgDx, imgDy, mPoseAffine, mCostAffineTexture);
	alignHomography(img, imgDx, imgDy, mPoseHomography, mCostHomographyTexture);

	//Corners
	std::vector<Eigen::Vector2f> refCorners;
	refCorners.push_back(Eigen::Vector2f(0, 0));
	refCorners.push_back(Eigen::Vector2f(0, mRefImg.rows));
	refCorners.push_back(Eigen::Vector2f(mRefImg.cols, mRefImg.rows));
	refCorners.push_back(Eigen::Vector2f(mRefImg.cols, 0));

	mCornerPosAffine.resize(4);
	mCornerPosHomography.resize(4);
	for (int i = 0; i < 4; i++)
	{
		mCornerPosAffine[i] = (mPoseAffine.inverse() * refCorners[i].homogeneous()).eval().hnormalized();
		mCornerPosHomography[i] = (mPoseHomography.inverse() * refCorners[i].homogeneous()).eval().hnormalized();
	}
}

void TestWindow::resize()
{
	mTiler.configDevice(Eigen::Vector2i::Zero(), UserInterfaceInfo::Instance().getScreenSize(), 2, 2);
	mTiler.fillTiles();

	float scale = 1.5f;
	Eigen::Vector2i sizeScaled = (scale*mImageSize.cast<float>()).cast<int>();
	Eigen::Vector2f padding = (sizeScaled - mImageSize).cast<float>();
	Eigen::Matrix4f offsetMat;
	offsetMat << 1, 0, 0, padding[0] / 2, 0, 1, 0, padding[1] / 2, 0, 0, 1, 0, 0, 0, 0, 1;

	mTiler.setImageMVP(0, sizeScaled);
	mTiler.multiplyMVP(0, offsetMat);

	mTiler.setImageMVP(1, sizeScaled);
	mTiler.multiplyMVP(1, offsetMat);

	mTiler.setImageMVP(2, mImageSize);
	mTiler.setImageMVP(3, mImageSize);
}

void TestWindow::draw()
{
	//Generic vertices
	std::vector<Eigen::Vector4f> vertices;
	std::vector<Eigen::Vector2f> textureCoords;
	TextureShader::CreateVertices(mImageSize, vertices, textureCoords);

	///////////////////////////
	// Affine
	// Alignment
	mTiler.setActiveTile(0);
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());
	mShaders->getTextureWarp().setMVPMatrix(mTiler.getMVP());

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	mShaders->getTextureWarp().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), Eigen::Matrix3fr::Identity(), StaticColors::Green(), mImageSize);
	mShaders->getTextureWarp().renderTexture(mImgTexture.getTarget(), mImgTexture.getId(), mPoseAffine, StaticColors::Red(), mImageSize);

	//mShaders->getTexture().renderTexture(mEvalPositionsTexture.getTarget(), mEvalPositionsTexture.getId(), eutils::FromSize(mEvalPositionsTexture.getSize()), 0.3f);
	glDisable(GL_BLEND);

	mShaders->getColor().drawVertices(GL_LINE_LOOP, mCornerPosAffine.data(), mCornerPosAffine.size(), StaticColors::Green());

	//Cost
	mTiler.setActiveTile(2);
	mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTexture().renderTexture(mCostAffineTexture.getTarget(), mCostAffineTexture.getId(), eutils::FromSize(mCostAffineTexture.getSize()));


	///////////////////////////
	// Homography
	// Alignment
	mTiler.setActiveTile(1);
	mShaders->getTextureWarp().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	mShaders->getTextureWarp().renderTexture(mRefTexture.getTarget(), mRefTexture.getId(), Eigen::Matrix3fr::Identity(), StaticColors::Green(), mImageSize);
	mShaders->getTextureWarp().renderTexture(mImgTexture.getTarget(), mImgTexture.getId(), mPoseHomography, StaticColors::Red(), mImageSize);
	glDisable(GL_BLEND);

	mShaders->getColor().drawVertices(GL_LINE_LOOP, mCornerPosHomography.data(), mCornerPosHomography.size(), StaticColors::Green());


	//Cost
	mTiler.setActiveTile(3);
	mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getTexture().renderTexture(mCostHomographyTexture.getTarget(), mCostHomographyTexture.getId(), eutils::FromSize(mCostHomographyTexture.getSize()));
}

} /* namespace dtslam */
