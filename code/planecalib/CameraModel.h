
#ifndef CAMERAMODEL_H_
#define CAMERAMODEL_H_

#include <memory>
#include <opencv2/core.hpp>
#include <gflags/gflags.h>
#include "CameraDistortionModel.h"
#include "log.h"

namespace planecalib
{

//Forward
template<class TDistortionModel>
class CameraModel_;

//This typedef decides which camera model to use in the system
//It has to be a typedef because ceres requires templated functions
//to build the automatic diff cost functions and templated methods cannot be virtual.
typedef CameraModel_<DivisionDistortionModel> CameraModel;

template<class TDistortionModel_>
class CameraModel_
{
public:
	CameraModel_()
	{
	}

	typedef TDistortionModel_ TDistortionModel;

	void init(const Eigen::Vector2f &principalPoint, const Eigen::Vector2f &focalLengths, Eigen::Vector2i &imageSize)
	{
		mPrincipalPoint = principalPoint;
		mFocalLengths = focalLengths;
		mImageSize = imageSize;
	}

	const Eigen::Vector2f &getPrincipalPoint() const { return mPrincipalPoint; }
	Eigen::Vector2f &getPrincipalPoint() { return mPrincipalPoint; }
	
	const Eigen::Vector2f &getFocalLength() const { return mFocalLengths; }
	Eigen::Vector2f &getFocalLength() { return mFocalLengths; }
	
	static const int kParamCount=4;
	Eigen::Vector4d getParams() const { return Eigen::Vector4d(mPrincipalPoint[0], mPrincipalPoint[1], mFocalLengths[0], mFocalLengths[1]); }
	void setParams(const Eigen::Vector4d &params) 
	{ 
		mPrincipalPoint[0] = (float)params[0];
		mPrincipalPoint[1] = (float)params[1];
		mFocalLengths[0] = (float)params[2];
		mFocalLengths[1] = (float)params[3];
	}

	const Eigen::Vector2i &getImageSize() const { return mImageSize; }
	TDistortionModel &getDistortionModel() {return mDistortionModel;}
	const TDistortionModel &getDistortionModel() const {return mDistortionModel;}

	void setFromK(const Eigen::Matrix3fr &K)
	{
		mPrincipalPoint = Eigen::Vector2f(K(0, 2), K(1, 2));
		mFocalLengths = Eigen::Vector2f(K(0, 0), K(1, 1));
	}

	void initLUT();

	bool isPointInside(float depth, const Eigen::Vector2f &p) const
	{
		return depth > 0 &&  //Depth is positive
			p[0] >= 0 && p[1] >= 0 && p[0] < mImageSize[0] && p[1] < mImageSize[1]; //Inside image
	}

	//////////////////////////////////////
	// Project form world
	// Same code, one using return value, one using args by-reference (for ceres)
	Eigen::Vector2f projectFromWorld(const Eigen::Vector3f &xc) const
	{
		const Eigen::Vector2f xn = xc.hnormalized();
		Eigen::Vector2f uv(xn[0] * mFocalLengths[0], xn[1] * mFocalLengths[1]);
		return mDistortionModel.apply(uv) + mPrincipalPoint;
	}
	
	template <class TPointMatA, class TPointMatB>
	void projectFromWorld(const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &p) const
	{
		typedef typename TPointMatA::Scalar TScalar;
		Eigen::Matrix<TScalar, 2, 1> xn = x.hnormalized();
		xn[0] *= mFocalLengths[0];
		xn[1] *= mFocalLengths[1];
		mDistortionModel.apply(xn,xd);
		p += mPrincipalPoint;
	}

	template <class TCameraParams, class TDistortionParams, class TPointMatA, class TPointMatB>
	static void ProjectFromWorld(const Eigen::MatrixBase<TCameraParams> &cameraParams, const Eigen::MatrixBase<TDistortionParams> &distortionParams, const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &p)
	{
		typedef typename TPointMatA::Scalar TScalar;
		Eigen::Matrix<TScalar, 2, 1> xd;
		TDistortionModel::DistortPoint(distortionParams, x.hnormalized(), xd);
		ProjectFromDistorted(cameraParams,xd, p);
	}

	void projectFromWorldJacobian(const Eigen::Vector3f &xc, Eigen::Vector3f &ujac, Eigen::Vector3f &vjac) const;
	void projectFromWorldJacobianLUT(const Eigen::Vector2i &uv, Eigen::Vector3f &ujac, Eigen::Vector3f &vjac) const
	{
		assert(uv[0] >= 0 && uv[0] < mProjectFromWorldJacobianLUT.cols() && uv[1] >= 0 && uv[1] < mProjectFromWorldJacobianLUT.rows());
		const Eigen::Matrix<float,3,2> &lut = mProjectFromWorldJacobianLUT(uv[1], uv[0]);
		ujac = lut.col(0);
		vjac = lut.col(1);
	}

	//////////////////////////////////////
	// Unproject (assuming z=1)
	// Note that often no analytical formula is present, therefore we have no templated version for ceres
	// Also, use integer pixel positions so that we can use a LUT for undistortion
	Eigen::Vector3f unprojectToWorld(const Eigen::Vector2f &uv) const
	{
		Eigen::Vector3f pn = mDistortionModel.applyInv(uv - mPrincipalPoint);
		pn[0] /= mFocalLengths[0];
		pn[1] /= mFocalLengths[1];
		return pn.homogeneous();
	}
	Eigen::Vector3f unprojectToWorldLUT(const Eigen::Vector2i &uv) const
	{
		//if (!(uv.x >= 0 && uv.x < mUnprojectLUT.cols && uv.y >= 0 && uv.y < mUnprojectLUT.rows))
		//{
			//DTSLAM_LOG << "Here!!!!!!!!!!!!!!!!!!!!!!!!!\n";
			assert(uv[0] >= 0 && uv[0] < mUnprojectLUT.cols() && uv[1] >= 0 && uv[1] < mUnprojectLUT.rows());
		//}
		return mUnprojectLUT(uv[1], uv[0]);
	}

protected:
	//Four parameters of the intrinsic matrix
	Eigen::Vector2f mPrincipalPoint;
	Eigen::Vector2f mFocalLengths;
	Eigen::Vector2i mImageSize;

	//Distortion model
	TDistortionModel mDistortionModel;

	Eigen::Matrix<Eigen::Vector3f,Eigen::Dynamic,Eigen::Dynamic> mUnprojectLUT; //Unproject doesn't need an analytical formula. If it is slow we can cache it in a LUT.
	Eigen::Matrix<Eigen::Matrix<float,3,2>, Eigen::Dynamic, Eigen::Dynamic> mProjectFromWorldJacobianLUT; //Can be cached if the pixel value is known
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Template implementations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class TDistortionModel>
void CameraModel_<TDistortionModel>::initLUT()
{
	MYAPP_LOG << "Building LUTs for image of size " << mImageSize.transpose() << "...";
	mUnprojectLUT.resize(mImageSize[1], mImageSize[0]);
	mProjectFromWorldJacobianLUT.resize(mImageSize[1], mImageSize[0]);
	for(int v=0; v<mImageSize[1]; v++)
	{
		for (int u = 0; u<mImageSize[0]; u++)
		{
			Eigen::Vector3f &itemUnprojectLUT = mUnprojectLUT(v,u);
			Eigen::Matrix<float,3,2> &itemProjectLUT = mProjectFromWorldJacobianLUT(v,u);

			itemUnprojectLUT = unprojectToWorld(Eigen::Vector2f(u, v));

			Eigen::Vector3f ujac, vjac;
			projectFromWorldJacobian(itemUnprojectLUT, ujac, vjac);
			itemProjectLUT << ujac, vjac;
		}
	}
	MYAPP_LOG << "done\n";
}

}

#endif /* CAMERAMODEL_H_ */
