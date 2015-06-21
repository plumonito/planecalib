#ifndef REPROJECTIONERROR3DIMPL_HPP_
#define REPROJECTIONERROR3DIMPL_HPP_

#include "CeresUtils.h"

namespace planecalib
{

template<class T>
void ReprojectionError3D::computeXc(const T * const _pp, const T * const _distortionParams, const T * const _focals, const T * const _xc, T *residuals) const
{
	Eigen::Map<Eigen::Matrix<T, 3, 1>> xc((T*)_xc);
	Eigen::Map<Eigen::Matrix<T, 2, 1>> pp((T*)_pp);
	Eigen::Map<Eigen::Matrix<T, TDistortionParamVector::RowsAtCompileTime, 1>> distortionParams((T*)_distortionParams);
	Eigen::Map<Eigen::Matrix<T, 2, 1>> focals((T*)_focals);

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
		CameraModel::ProjectFromWorld(pp, distortionParams, focals, xc, p);

		residuals[0] = (p[0] - T(mImgPoint[0])) / T(mScale);
		residuals[1] = (p[1] - T(mImgPoint[1])) / T(mScale);
	}
}

template<class T>
void ReprojectionError3D::evaluateWithRparams(const T * const pp, const T * const distortionParams, const T * const focals, const T * const rparams, const T * const t, const T * const x, T *residuals) const
{
	T xr[3];
	ceres::AngleAxisRotatePoint(rparams, x, xr);

	T xc[3];
	xc[0] = xr[0]+t[0];
	xc[1] = xr[1]+t[1];
	xc[2] = xr[2]+t[2];

	computeXc(pp,distortionParams,focals,xc,residuals);
}

template<class T>
void ReprojectionError3D::evaluateWithRmat(const T * const pp, const T * const distortionParams, const T * const focals, const T * const R, const T * const t, const T * const x, T *residuals) const
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

	computeXc(pp, distortionParams, focals, xc, residuals);
}

template<class T>
bool PoseReprojectionError3D::operator()(const T * const rparams, const T * const t, T *residuals) const
{
	const auto pp = mPP.cast<T>().eval();
	const auto distortion = mDistortionParams.cast<T>().eval();
	const auto focals = mFocals.cast<T>().eval();
	const auto x = mFeaturePosition.cast<T>().eval();

	evaluateWithRparams(pp.data(), distortion.data(), focals.data(), rparams, t, x.data(), residuals);
	return true;
}

template<class T>
bool BAReprojectionError3D::operator()(const T * const pp, const T * const distortionParams, const T * const focals, const T * const rparams, const T * const t, const T * const x, T *residuals) const
{
	evaluateWithRparams(pp, distortionParams, focals, rparams, t, x, residuals);
	return true;
}

}


#endif /* REPROJECTIONERROR3DIMPL_HPP_ */
