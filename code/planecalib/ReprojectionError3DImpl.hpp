#ifndef REPROJECTIONERROR3DIMPL_HPP_
#define REPROJECTIONERROR3DIMPL_HPP_

#include "CeresUtils.h"

namespace planecalib
{

template<class T>
void ReprojectionError3D::computeXc(const T * const _cameraParams, const T * const _distortionParams, const T * const _xc, T *residuals) const
{
	Eigen::Map<Eigen::Matrix<T, 3, 1>> xc((T*)_xc);
	Eigen::Map<Eigen::Matrix<T, CameraModel::kParamCount, 1>> cameraParams((T*)_cameraParams);
	Eigen::Map<Eigen::Matrix<T, CameraModel::TDistortionModel::kParamCount, 1>> distortionParams((T*)_distortionParams);

	if (CeresUtils::ToDouble(xc[2]) <= 0)
	{
		//Negative point
		residuals[0] = T(1e3);
		residuals[1] = T(1e3);
	}
	else
	{
		//Point in front of camera, proceed
		Eigen::Matrix<T,2,1> p;
		CameraModel::ProjectFromWorld(cameraParams, distortionParams,xc,p);

		residuals[0] = (p[0] - T(mImgPoint[0])) / T(mScale);
		residuals[1] = (p[1] - T(mImgPoint[1])) / T(mScale);
	}
}

template<class T>
void ReprojectionError3D::computeRparams(const T * const cameraParams, const T * const distortionParams, const T * const rparams, const T * const t, const T * const x, T *residuals) const
{
	T xr[3];
	ceres::AngleAxisRotatePoint(rparams, x, xr);

	T xc[3];
	xc[0] = xr[0]+t[0];
	xc[1] = xr[1]+t[1];
	xc[2] = xr[2]+t[2];

	computeXc(cameraParams,distortionParams,xc,residuals);
}

template<class T>
void ReprojectionError3D::computeRmat(const T * const cameraParams, const T * const distortionParams, const T * const R, const T * const t, const T * const x, T *residuals) const
{
	auto xwmat = CeresUtils::FixedRowMajorAdapter3x1(x);

	auto Rmat = CeresUtils::FixedRowMajorAdapter3x3(R);
	T xr[3];
	auto xrmat = CeresUtils::FixedRowMajorAdapter3x1(xr);
	CeresUtils::matrixMatrix(Rmat,xwmat,xrmat);

	T xc[3];
	xc[0] = xr[0]+t[0];
	xc[1] = xr[1]+t[1];
	xc[2] = xr[2]+t[2];

	computeXc(cameraParams,distortionParams,xc,residuals);
}

template<class T>
bool PoseReprojectionError3D::operator()(const T * const rparams, const T * const t, T *residuals) const
{
	Eigen::Matrix<T, CameraModel::kParamCount, 1> cameraParams = mCameraParams.cast<T>();
	Eigen::Matrix<T, CameraModel::TDistortionModel::kParamCount, 1> distortionParams = mDistortionParams.cast<T>();
	Eigen::Matrix<T,3,1> x = mFeaturePosition.cast<T>();

	computeRparams(cameraParams.data(), distortionParams.data(), rparams, t, x.data(), residuals);
	return true;
}

template<class T>
bool BAReprojectionError3D::operator()(const T * const cameraParams, const T * const distortionParams, const T * const rparams, const T * const t, const T * const x, T *residuals) const
{
	computeRparams(cameraParams,distortionParams,rparams, t, x, residuals);
	return true;
}

}


#endif /* REPROJECTIONERROR3DIMPL_HPP_ */
