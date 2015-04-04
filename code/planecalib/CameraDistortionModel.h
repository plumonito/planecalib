#ifndef CAMERADISTORTIONMODEL_H_
#define CAMERADISTORTIONMODEL_H_

#include <Eigen/Dense>
#include "eutils.h"
#include "cvutils.h"
#include "CeresUtils.h"

namespace planecalib
{

class NullCameraDistortionModel
{
public:
	NullCameraDistortionModel() {}
		
	Eigen::Matrix<float, 0, 1> getCoefficients() const { return Eigen::Matrix<float, 0, 1>(); }
	void setCoefficients(const Eigen::Matrix<float, 0, 1> &coeff) { }

	Eigen::Vector2f distortPoint(const Eigen::Vector2f &x) const
	{
		return x;
	}
	template <typename TScalar>
	void distortPoint(const Eigen::MatrixBase<Eigen::Matrix<TScalar, 2, 1>> &x, Eigen::MatrixBase<Eigen::Matrix<TScalar, 2, 1>> &xd) const
	{
		xd = x;
	}

	Eigen::Vector3f undistortPoint(const Eigen::Vector2f &pd) const
	{
		return pd.homogeneous();
	}
};

class RadialCameraDistortionModel
{
public:
	RadialCameraDistortionModel() {}

	void init(const Eigen::Vector2f &coeff, const Eigen::Vector2i imageSize)
	{
		init(coeff, imageSize.squaredNorm() / 4);
	}
	void init(const Eigen::Vector2f &coeff, float maxRadiusSq = 1e10)
	{
		mCoefficients = coeff;
		mMaxRadiusSq = maxRadiusSq;
	}

	Eigen::Vector2f getCoefficients() const { return mCoefficients; }
	void setCoefficients(const Eigen::Vector2f &coeff) { mCoefficients = coeff; }

	float getMaxRadiusSq() const {return mMaxRadiusSq;}
	void setMaxRadius(float maxRadiusSq)
	{
		mMaxRadiusSq = maxRadiusSq;
	}


	Eigen::Vector2f distortPoint(const Eigen::Vector2f &x) const
	{
		Eigen::Vector2f res;
		distortPoint(x, res);
		return res;
	}
	
	template <typename TScalar>
	void distortPoint(const Eigen::MatrixBase<Eigen::Matrix<TScalar, 2, 1>> &x, Eigen::MatrixBase<Eigen::Matrix<TScalar, 2, 1>> &xd) const
	{
		DistortPoint(mMaxRadiusSq, mCoefficients, x, xd);
	}

	template <typename TCoeff, typename TScalar>
	static void DistortPoint(double maxRadiusSq, const Eigen::MatrixBase<Eigen::Matrix<TCoeff, 2, 1>> &coeff, const Eigen::MatrixBase<Eigen::Matrix<TScalar, 2, 1>> &x, Eigen::MatrixBase<Eigen::Matrix<TScalar, 2, 1>> &xd)
	{
		TScalar r2 = x.squaredNorm();
		if (CeresUtils::ToDouble(r2) > maxRadiusSq)
			r2 = TScalar(maxRadiusSq);

		const TScalar r4 = r2*r2;
		const TScalar factor = TScalar(1) + TScalar(coeff[0])*r2 + TScalar(coeff[1])*r4;
		xd = factor*x;
	}

	//Non-linear, uses ceres
	Eigen::Vector2f undistortPoint(const Eigen::Vector2f &pd) const;

	//Create inverse model
	RadialCameraDistortionModel createInverseModel() const;

protected:
	Eigen::Vector2f mCoefficients;
	float mMaxRadiusSq; //Do not apply distortion after this radius. Keeps the distortion within the image limits where it was calibrated.
};
} 

#endif /* CAMERADISTORTIONMODEL_H_ */
