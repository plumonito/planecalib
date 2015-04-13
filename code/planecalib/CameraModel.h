
#ifndef CAMERAMODEL_H_
#define CAMERAMODEL_H_

#include <memory>
#include <opencv2/core.hpp>
#include <gflags/gflags.h>
#include <ceres/rotation.h>
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
typedef CameraModel_<RadialCameraDistortionModel> CameraModel;

template<class TDistortionModel_>
class CameraModel_
{
public:
	CameraModel_()
	{
	}

	typedef TDistortionModel_ TDistortionModel;

	void init(float fx, float fy, float u0, float v0, int width, int height)
	{
		mFx = fx;
		mFy = fy;
		mU0 = u0;
		mV0 = v0;
		mImageSize = Eigen::Vector2i(width, height);
	}

	float getFx() const {return mFx;}
	float getFy() const {return mFy;}
	float getU0() const {return mU0;}
	float getV0() const {return mV0;}
	
	static const int kParamCount=4;
	Eigen::Vector4d getParams() const { return Eigen::Vector4d(mFx,mFy,mU0,mV0); }
	void setParams(const Eigen::Vector4d &params) 
	{ 
		mFx = (float)params[0]; 
		mFy = (float)params[1];
		mU0 = (float)params[2];
		mV0 = (float)params[3];
	}

	const Eigen::Vector2i &getImageSize() const { return mImageSize; }
	TDistortionModel &getDistortionModel() {return mDistortionModel;}
	const TDistortionModel &getDistortionModel() const {return mDistortionModel;}

	float getMaxRadiusSq(const Eigen::Vector2i &imageSize) const;

	Eigen::Matrix3fr getK() const 
	{ 
		Eigen::Matrix3fr k; 
		k << mFx, 0, mU0, 0, mFy, mV0, 0, 0, 1; 
		return k; 
	}

	void initLUT();

	bool isPointInside(const Eigen::Vector3f &xc, const Eigen::Vector2f &p) const
	{
		return xc[2] > 0 &&  //Depth is positive
			p[0] >= 0 && p[1] >= 0 && p[0] < mImageSize[0] && p[1] < mImageSize[1]; //Inside image
	}

	//////////////////////////////////////
	// Project form world
	// Same code, one using return value, one using args by-reference (for ceres)
	Eigen::Vector2f projectFromWorld(const Eigen::Vector3f &xc) const
	{
		return projectFromDistorted(mDistortionModel.distortPoint(xc.hnormalized()));
	}
	
	template <class TPointMatA, class TPointMatB>
	void projectFromWorld(const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &p) const
	{
		typedef typename TPointMatA::Scalar TScalar;
		Eigen::Matrix<TScalar, 2, 1> xd;
		mDistortionModel.distortPoint(x.hnormalized().eval(),xd);
		projectFromDistorted(xd,p);
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
		return mDistortionModel.undistortPoint(unprojectToDistorted(uv)).homogeneous();
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
	float mFx;
	float mFy;
	float mU0;
	float mV0;
	Eigen::Vector2i mImageSize;

	//Distortion model
	TDistortionModel mDistortionModel;

	Eigen::Matrix<Eigen::Vector3f,Eigen::Dynamic,Eigen::Dynamic> mUnprojectLUT; //Unproject doesn't need an analytical formula. If it is slow we can cache it in a LUT.
	Eigen::Matrix<Eigen::Matrix<float,3,2>, Eigen::Dynamic, Eigen::Dynamic> mProjectFromWorldJacobianLUT; //Can be cached if the pixel value is known

	//////////////////////////////////////
	// Project form distorted
	// Same code, one using return value, one using args by-reference (for ceres)
	Eigen::Vector2f projectFromDistorted(const Eigen::Vector2f &pd) const
	{
		return Eigen::Vector2f(mFx*pd[0] + mU0, mFy*pd[1] + mV0);
	}

	template <class TPointMatA, class TPointMatB>
	void projectFromDistorted(const Eigen::MatrixBase<TPointMatA> &xd, Eigen::MatrixBase<TPointMatB> &p) const
	{
		typedef typename TPointMatA::Scalar T;
		p[0] = T(mFx)*xd[0] + T(mU0);
		p[1] = T(mFy)*xd[1] + T(mV0);
	}
	template <class TParams, class TPointMatA, class TPointMatB>
	static void ProjectFromDistorted(const Eigen::MatrixBase<TParams> &cameraParams, const Eigen::MatrixBase<TPointMatA> &xd, Eigen::MatrixBase<TPointMatB> &p)
	{
		static_assert(TParams::SizeAtCompileTime == 4, "Params must be of size 4");

		auto &fx = cameraParams[0];
		auto &fy = cameraParams[1];
		auto &u0 = cameraParams[2];
		auto &v0 = cameraParams[3];
		
		p[0] = fx*xd[0] + u0;
		p[1] = fy*xd[1] + v0;
	}

	Eigen::Vector2f unprojectToDistorted(const Eigen::Vector2f &uv) const
	{
		return Eigen::Vector2f((uv[0] - mU0) / mFx, (uv[1] - mV0) / mFy);
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Template implementations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class TDistortionModel>
float CameraModel_<TDistortionModel>::getMaxRadiusSq(const Eigen::Vector2i &imageSize) const
{
	Eigen::Vector2f corners[] = { Eigen::Vector2f(0, 0), Eigen::Vector2f(0, imageSize[1]),
		Eigen::Vector2f(imageSize[0], imageSize[1]), Eigen::Vector2f(imageSize[0], 0) };

	float maxRadiusSq = 0;
	for(int i=0; i<4; ++i)
	{
		const Eigen::Vector2f xn = unprojectToWorld(corners[i]).hnormalized();
		const float r2 = xn.squaredNorm();
		if(r2>maxRadiusSq)
			maxRadiusSq = r2;
	}

	return maxRadiusSq;
}

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
