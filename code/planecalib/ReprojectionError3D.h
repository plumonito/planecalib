#ifndef POSEREPROJECTIONERROR3D_H_
#define POSEREPROJECTIONERROR3D_H_

#include <vector>
#include <Eigen/Dense>
#include "CameraModel.h"
#include "CeresUtils.h"
#include "PoseEstimationCommon.h"
#include "Map.h"
#include "FeatureMatcher.h"

namespace planecalib
{

class ReprojectionError3D
{
public:
	ReprojectionError3D(const FeatureMeasurement &m):
		ReprojectionError3D(m.getOctave(), m.getPosition().cast<double>())
	{
	}

	ReprojectionError3D(const int octave, const Eigen::Vector2d &imgPoint) :
		mScale(1<<octave), mImgPoint(imgPoint)
	{
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	template<class T>
    void computeRparams(const T * const cameraParams, const T * const distortionParams, const T * const rparams, const T * const t, const T * const x, T *residuals) const;

	template<class T>
	void computeRmat(const T * const cameraParams, const T * const distortionParams, const T * const R, const T * const t, const T * const x, T *residuals) const;

	double evalToDistanceSq(const Eigen::VectorXd &cameraParams, const Eigen::VectorXd &distortionParams, const Eigen::Matrix3dr &R, const Eigen::Vector3d &t, const Eigen::Vector3d &x) const;
	double evalToDistanceSq(const Eigen::VectorXd &cameraParams, const Eigen::VectorXd &distortionParams, const Eigen::Vector3d &Rparams, const Eigen::Vector3d &t, const Eigen::Vector3d &x) const;

protected:
	const int mScale;
	const Eigen::Vector2d mImgPoint;

	template<class T>
	void computeXc(const T * const cameraParams, const T * const distortionParams, const T * const xc, T *residuals) const;
};

class PoseReprojectionError3D: public ReprojectionError3D
{
public:
	PoseReprojectionError3D(const CameraModel *camera, const FeatureMatch &match) :
		PoseReprojectionError3D(camera, match.getFeature().mPosition3D.cast<double>(), match.getOctave(), match.getPosition().cast<double>())
	{
	}

	PoseReprojectionError3D(const CameraModel * const camera, const Eigen::Vector3d &featurePosition, const int octave, const Eigen::Vector2d &imgPoint) :
		ReprojectionError3D(octave, imgPoint), mCameraParams(camera->getParams()), mDistortionParams(camera->getDistortionModel().getParams()), mFeaturePosition(featurePosition)
	{
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	const Eigen::Vector3d &getFeaturePosition() const { return mFeaturePosition; }

	template<class T>
    bool operator()(const T * const rparams, const T * const t, T *residuals) const;

	double evalToDistanceSq(const Eigen::Matrix3dr &R, const Eigen::Vector3d &t) const
	{
		return ReprojectionError3D::evalToDistanceSq(mCameraParams, mDistortionParams, R,t,mFeaturePosition);
	}

	double evalToDistanceSq(const Eigen::Vector3d &Rparams, const Eigen::Vector3d &t) const
	{
		return ReprojectionError3D::evalToDistanceSq(mCameraParams, mDistortionParams, Rparams, t, mFeaturePosition);
	}

protected:
	const Eigen::VectorXd mCameraParams;
	const Eigen::VectorXd mDistortionParams;
	const Eigen::Vector3d mFeaturePosition;
};

class BAReprojectionError3D: public ReprojectionError3D
{
public:
	BAReprojectionError3D(const int octave, const Eigen::Vector2d &imgPoint):
		ReprojectionError3D(octave, imgPoint)
	{
	}

	BAReprojectionError3D(const FeatureMeasurement &measurement):
		BAReprojectionError3D(measurement.getOctave(), measurement.getPosition().cast<double>())
	{
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	template<class T>
	bool operator()(const T * const cameraParams, const T * const distortionParams, const T * const rparams, const T * const t, const T * const x, T *residuals) const;

protected:
};

}

//Include implementation of templates
#include "ReprojectionError3DImpl.hpp"

#endif /* POSEREPROJECTIONERROR3D_H_ */
