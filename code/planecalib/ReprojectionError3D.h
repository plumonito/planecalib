#ifndef POSEREPROJECTIONERROR3D_H_
#define POSEREPROJECTIONERROR3D_H_

#include <vector>
#include <Eigen/Dense>
#include "CameraModel.h"
#include "PoseEstimationCommon.h"
#include "Map.h"
#include "FeatureMatcher.h"

namespace planecalib
{

class ReprojectionError3D
{
public:
	typedef CameraModel::TDistortionModel::TParamVector TDistortionParamVector;

	ReprojectionError3D(const FeatureMeasurement &m) :
		ReprojectionError3D(m.getPosition().cast<double>(), 1 << m.getOctave())
	{
	}

	ReprojectionError3D(const Eigen::Vector2d &imgPoint, const double scale) :
		mScale(scale), mImgPoint(imgPoint)
	{
	}
	
	~ReprojectionError3D() {}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	template<class T>
	void evaluateWithRparams(const T * const pp, const T * const distortionParams, const T * const focals, const T * const rparams, const T * const t, const T * const x, T *residuals) const;

	void evaluateWithRparams(const Eigen::Vector2d &pp, const TDistortionParamVector &distortion, const Eigen::Vector2d &focals, const Eigen::Vector3d &rparams, const Eigen::Vector3d &t, const Eigen::Vector3d &x, Eigen::Vector2d &residuals) const
	{
		evaluateWithRparams(pp.data(), distortion.data(), focals.data(), rparams.data(), t.data(), x.data(), residuals.data());
	}


	template<class T>
	void evaluateWithRmat(const T * const pp, const T * const distortionParams, const T * const focals, const T * const R, const T * const t, const T * const x, T *residuals) const;

	void evaluateWithRmat(const Eigen::Vector2d &pp, const TDistortionParamVector &distortion, const Eigen::Vector2d &focals, const Eigen::Matrix3dr &R, const Eigen::Vector3d &t, const Eigen::Vector3d &x, Eigen::Vector2d &residuals) const
	{
		evaluateWithRmat(pp.data(), distortion.data(), focals.data(), R.data(), t.data(), x.data(), residuals.data());
	}

protected:
	const double mScale;
	const Eigen::Vector2d mImgPoint;

	template<class T>
	void computeXc(const T * const pp, const T * const distortionParams, const T * const focals, const T * const xc, T *residuals) const;
};

class PoseReprojectionError3D: public ReprojectionError3D
{
public:
	PoseReprojectionError3D(const CameraModel *camera, const FeatureMatch &match) :
		PoseReprojectionError3D(camera, match.getFeature().mPosition3D.cast<double>(), match.getPosition().cast<double>(), 1<<match.getOctave())
	{
	}

	PoseReprojectionError3D(const CameraModel * const camera, const Eigen::Vector3d &featurePosition, const Eigen::Vector2d &imgPoint, const double scale) :
		ReprojectionError3D(imgPoint, scale), mPP(camera->getPrincipalPoint().cast<double>()), mDistortionParams(camera->getDistortionModel().getParams()), mFocals(camera->getFocalLength().cast<double>()), mFeaturePosition(featurePosition)
	{
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	const Eigen::Vector3d &getFeaturePosition() const { return mFeaturePosition; }

	template<class T>
    bool operator()(const T * const rparams, const T * const t, T *residuals) const;

	bool operator()(const Eigen::Vector3d &rparams, const Eigen::Vector3d &t, Eigen::Vector2d &residuals) const
	{
		return (*this)(rparams.data(), t.data(), residuals.data());
	}

	void evaluateWithRmat(const Eigen::Matrix3dr &R, const Eigen::Vector3d &t, Eigen::Vector2d &residuals) const
	{
		ReprojectionError3D::evaluateWithRmat(mPP, mDistortionParams, mFocals, R, t, mFeaturePosition, residuals);
	}

protected:
	const Eigen::Vector2d mPP;
	const TDistortionParamVector mDistortionParams;
	const Eigen::Vector2d mFocals;
	const Eigen::Vector3d mFeaturePosition;
};

class BAReprojectionError3D: public ReprojectionError3D
{
public:
	BAReprojectionError3D(const int octave, const Eigen::Vector2d &imgPoint):
		ReprojectionError3D(imgPoint, 1 << octave)
	{
	}

	BAReprojectionError3D(const FeatureMeasurement &measurement):
		BAReprojectionError3D(measurement.getOctave(), measurement.getPosition().cast<double>())
	{
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	template<class T>
	bool operator()(const T * const pp, const T * const distortionParams, const T * const focals, const T * const rparams, const T * const t, const T * const x, T *residuals) const;

protected:
};

}

//Include implementation of templates
#include "ReprojectionError3DImpl.hpp"

#endif /* POSEREPROJECTIONERROR3D_H_ */
