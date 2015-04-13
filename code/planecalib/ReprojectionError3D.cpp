
#include "ReprojectionError3D.h"

namespace planecalib
{

double ReprojectionError3D::evalToDistanceSq(const Eigen::VectorXd &cameraParams, const Eigen::VectorXd &distortionParams, const Eigen::Matrix3dr &R, const Eigen::Vector3d &t, const Eigen::Vector3d &x) const
{
	Eigen::Vector2d residuals;

	computeRmat(cameraParams.data(), distortionParams.data(), R.data(), t.data(), x.data(), residuals.data());
	return residuals.squaredNorm();
}

double ReprojectionError3D::evalToDistanceSq(const Eigen::VectorXd &cameraParams, const Eigen::VectorXd &distortionParams, const Eigen::Vector3d &Rparams, const Eigen::Vector3d &t, const Eigen::Vector3d &x) const
{
	Eigen::Vector2d residuals;

	computeRparams(cameraParams.data(), distortionParams.data(), Rparams.data(), t.data(), x.data(), residuals.data());
	return residuals.squaredNorm();
}

}
