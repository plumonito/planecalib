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

	//Distortion
	Eigen::Vector2f distortPoint(const Eigen::Vector2f &x) const
	{
		return x;
	}

	template <typename TScalar>
	void distortPoint(const Eigen::MatrixBase<Eigen::Matrix<TScalar, 2, 1>> &x, Eigen::MatrixBase<Eigen::Matrix<TScalar, 2, 1>> &xd) const
	{
		xd = x;
	}

	Eigen::Vector2f undistortPoint(const Eigen::Vector2f &pd) const
	{
		return pd;
	}
};

class RadialCameraDistortionModel
{
public:
	RadialCameraDistortionModel() {}

	template<class TSize>
	static float MaxRadiusSqFromImageSize(const Eigen::Matrix<TSize,2,1> &imageSize)
	{
		return static_cast<float>(imageSize.squaredNorm());
	}

	template<class TSize>
	void init(const Eigen::Vector2f &coeff, const Eigen::Matrix<TSize,2,1> &imageSize)
	{
		init(coeff, MaxRadiusSqFromImageSize(imageSize));
	}
	void init(const Eigen::Vector2f &coeff, float maxRadiusSq = 1e10)
	{
		mCoefficients = coeff;
		mMaxRadiusSq = maxRadiusSq;
	}

	const Eigen::Vector2f &getCoefficients() const { return mCoefficients; }
	void setCoefficients(const Eigen::Vector2f &coeff) { mCoefficients = coeff; }

	static const int kParamCount = 2;
	Eigen::Vector2d getParams() const { return mCoefficients.cast<double>(); }
	void setParams(const Eigen::Vector2d &params) { mCoefficients = params.cast<float>(); }

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
	
	template <class TPointMatA, class TPointMatB>
	void distortPoint(const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &xd) const
	{
		DistortPoint(mMaxRadiusSq, mCoefficients, x, xd);
	}

	template <class TCoeffMat, class TPointMatA, class TPointMatB>
	static void DistortPoint(double maxRadiusSq, const Eigen::MatrixBase<TCoeffMat> &coeff, const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &xd)
	{
		static_assert(TCoeffMat::SizeAtCompileTime==2, "Param coeff must be of size 2");
		static_assert(TPointMatA::SizeAtCompileTime == 2, "Param x must be of size 2");
		static_assert(TPointMatB::SizeAtCompileTime == 2, "Param xd must be of size 2");

		typedef typename TPointMatA::Scalar TScalar;

		TScalar r2 = x.squaredNorm();
		if (CeresUtils::ToDouble(r2) > maxRadiusSq)
			r2 = TScalar(maxRadiusSq);

		const TScalar r4 = r2*r2;
		const TScalar factor = TScalar(1) + TScalar(coeff[0])*r2 + TScalar(coeff[1])*r4;
		xd = factor*x;
	}

	template <class TCoeffMat, class TPointMatA, class TPointMatB>
	static void DistortPoint(const Eigen::MatrixBase<TCoeffMat> &coeff, const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &xd)
	{
		static_assert(TCoeffMat::SizeAtCompileTime == 2, "Param coeff must be of size 2");
		static_assert(TPointMatA::SizeAtCompileTime == 2, "Param x must be of size 2");
		static_assert(TPointMatB::SizeAtCompileTime == 2, "Param xd must be of size 2");

		typedef typename TPointMatA::Scalar TScalar;

		TScalar r2 = x.squaredNorm();
		const TScalar r4 = r2*r2;
		const TScalar factor = TScalar(1) + TScalar(coeff[0])*r2 + TScalar(coeff[1])*r4;
		xd = factor*x;
	}


	//Non-linear, uses ceres
	Eigen::Vector2f undistortPoint(const Eigen::Vector2f &pd) const;

	//Create inverse model
	//RadialCameraDistortionModel createInverseModel() const;

protected:
	Eigen::Vector2f mCoefficients;
	float mMaxRadiusSq; //Do not apply distortion after this radius. Keeps the distortion within the image limits where it was calibrated.
};

///////////////////////////////////////////////////////
// DivisionDistortionModel
class DivisionDistortionModel
{
public:
	DivisionDistortionModel() {}

	void init(const float lambda, const Eigen::Vector2i &imageSize)
	{
		init(lambda, imageSize.squaredNorm());
	}
	void init(const float lambda, float maxRadiusSq = 1e50)
	{
		mLambda = lambda;
		mMaxRadiusSq = maxRadiusSq;
	}

	float getLambda() const { return mLambda; }
	void setLambda(float lambda) { mLambda = lambda; }

	static const int kParamCount = 1;
	Eigen::Matrix<float, 1, 1> getCoefficients() const { Eigen::Matrix<float, 0, 1> p; p[0] = mLambda; return p; }
	void setCoefficients(const Eigen::Matrix<float, 1, 1> &coeff) { mLambda = coeff[0]; }

	float getMaxRadiusSq() const { return mMaxRadiusSq; }
	void setMaxRadius(float maxRadiusSq)
	{
		mMaxRadiusSq = maxRadiusSq;
	}


	Eigen::Vector2f apply(const Eigen::Vector2f &x) const
	{
		Eigen::Vector2f res;
		apply(x, res);
		return res;
	}

	//du = [du/dx, du/dy, du/dlambda], dv = [dv/dx, dv/dy, dv/dlambda]
	static void ApplyJacobian(float maxRadiusSq, float lambda, const Eigen::Vector2f &x, Eigen::Vector3f &du, Eigen::Vector3f &dv)
	{
		Eigen::Vector2f jac;

		float r2 = x.squaredNorm();
		if (r2 > maxRadiusSq)
		{
			const float factor = 1 + lambda*maxRadiusSq;
			dv[0] = 1 / factor;
			dv[1] = 0;
			dv[2] = 0;
			
			du[0] = 0;
			du[1] = 1 / factor;
			du[2] = 0;
		}
		else
		{
			const float factor = 1/(1 + lambda*r2);
			const float factor2 = factor*factor;
			du[0] = (1 + lambda*(x[1] * x[1] - x[0] * x[0]))*factor2;
			du[1] = -2*lambda*x[0]*x[1]*factor2;
			du[2] = -x[0] * r2*factor2;

			dv[0] = -2 * lambda*x[0] * x[1] * factor2;
			dv[1] = (1 + lambda*(x[0] * x[0] - x[1] * x[1]))*factor2;
			dv[2] = -x[1] * r2*factor2;
		}
	}

	template <class TPointMatA, class TPointMatB>
	void apply(const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &xd) const
	{
		Apply(mMaxRadiusSq, mLambda, x, xd);
	}

	template <class TLambda, class TPointMatA, class TPointMatB>
	static void Apply(double maxRadiusSq, const TLambda lambda, const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &xd)
	{
		static_assert(TPointMatA::SizeAtCompileTime == 2, "Param x must be of size 2");
		static_assert(TPointMatB::SizeAtCompileTime == 2, "Param xd must be of size 2");

		typedef typename TPointMatA::Scalar TScalar;

		TScalar r2 = x.squaredNorm();
		if (CeresUtils::ToDouble(r2) > maxRadiusSq)
			r2 = TScalar(maxRadiusSq);

		const TScalar factor = TScalar(1) + TScalar(lambda)*r2;
		xd[0] = x[0] / factor;
		xd[1] = x[1] / factor;
	}

	template <class TLambda, class TPointMatA, class TPointMatB>
	static void Apply(const TLambda lambda, const Eigen::MatrixBase<TPointMatA> &x, Eigen::MatrixBase<TPointMatB> &xd)
	{
		static_assert(TPointMatA::SizeAtCompileTime == 2, "Param x must be of size 2");
		static_assert(TPointMatB::SizeAtCompileTime == 2, "Param xd must be of size 2");

		typedef typename TPointMatA::Scalar TScalar;

		TScalar r2 = x.squaredNorm();
		const TScalar factor = TScalar(1) + TScalar(lambda)*r2;
		xd[0] = x[0] / factor;
		xd[1] = x[1] / factor;
	}


	//Non-linear
	Eigen::Vector2f applyInv(const Eigen::Vector2f &pd) const;

protected:
	float mLambda;
	float mMaxRadiusSq; //Do not apply distortion after this radius. Keeps the distortion within the image limits where it was calibrated.
};
}

#endif /* CAMERADISTORTIONMODEL_H_ */
