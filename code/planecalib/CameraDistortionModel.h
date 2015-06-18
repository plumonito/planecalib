#ifndef CAMERADISTORTIONMODEL_H_
#define CAMERADISTORTIONMODEL_H_

#include <Eigen/Dense>
#include "eutils.h"
#include "cvutils.h"
#include "CeresUtils.h"

namespace planecalib
{

/*
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
*/

///////////////////////////////////////////////////////
// DivisionDistortionModel
class DivisionDistortionModel
{
public:
	DivisionDistortionModel() {}

	void init(const float lambda=0)
	{
		mLambda = lambda;
	}

	float getLambda() const { return mLambda; }
	void setLambda(float lambda) { mLambda = lambda; }

	typedef Eigen::Matrix<double, 1, 1> TParamVector;

	TParamVector getParams() const { TParamVector p; p[0] = mLambda; return p; }
	void setParams(const TParamVector &coeff) { mLambda = (float)coeff[0]; }

	///////////////////////////
	// Apply 

	template <class TParamsMat, class TXMat, class TXdMat>
	static void EvaluateCeres(const Eigen::MatrixBase<TParamsMat> &params, const Eigen::MatrixBase<TXMat> &x, Eigen::MatrixBase<TXdMat> &xd, double **jacobians)
	{
		static_assert(TParamsMat::SizeAtCompileTime == TParamVector::SizeAtCompileTime, "Params vector is wrong size");
		const auto &lambda = params[0];

		EvaluateCeres(lambda, x, xd, jacobians);
	}

	template <class TLambda, class TXMat, class TXdMat>
	static void EvaluateCeres(const TLambda &lambda, const Eigen::MatrixBase<TXMat> &x, Eigen::MatrixBase<TXdMat> &xd, double **jacobians)
	{
		static_assert(TXMat::SizeAtCompileTime == 2, "Param x must be of size 2");
		static_assert(TXdMat::SizeAtCompileTime == 2, "Param xd must be of size 2");

		typedef typename TXMat::Scalar TScalar;

		const TScalar r2 = x.squaredNorm();
		const TScalar factor = TScalar(1) + TScalar(lambda)*r2;

		xd[0] = x[0] / factor;
		xd[1] = x[1] / factor;

		//Jacobian
		if (jacobians)
		{
			EvaluateCeresJacobians(lambda, x, r2, factor, jacobians);
		}
	}

	Eigen::Vector2f apply(const Eigen::Vector2f &x) const
	{
		Eigen::Vector2f res;
		EvaluateCeres(mLambda, x, res, NULL);
		return res;
	}

	///////////////////////////
	// Apply Inverse
	template <class TParamsMat, class TXMat, class TXdMat>
	static void EvaluateInvCeres(const Eigen::MatrixBase<TParamsMat> &params, const Eigen::MatrixBase<TXdMat> &xd, Eigen::MatrixBase<TXMat> &x, double **jacobians)
	{
		static_assert(TParamsMat::SizeAtCompileTime == TParamVector::SizeAtCompileTime, "Params vector is wrong size");
		const auto &lambda = params[0];

		EvaluateInvCeres(lambda, xd, x, jacobians);
	}
	template <class TLambda, class TXMat, class TXdMat>
	static void EvaluateInvCeres(const TLambda &lambda, const Eigen::MatrixBase<TXdMat> &xd, Eigen::MatrixBase<TXMat> &x, double **jacobians)
	{
		static_assert(TXMat::SizeAtCompileTime == 2, "Param x must be of size 2");
		static_assert(TXdMat::SizeAtCompileTime == 2, "Param xd must be of size 2");

		typedef typename TXMat::Scalar TScalar;

		const TScalar r2 = x.squaredNorm();
		const TScalar factor = TScalar(1) + TScalar(lambda)*r2;

		xd[0] = x[0] / factor;
		xd[1] = x[1] / factor;

		//Iterative
		const int kMaxIters = 11;
		Eigen::Matrix<TScalar,2,1> pn = pd;
		for (int j = 0; j < kMaxIters; j++)
		{
			TScalar r2 = pn.squaredNorm();
			TScalar factorInv = 1 + r2*lambda;
			pn = factorInv*pd;
		}

		x = pn;

		//Jacobian
		if (jacobians)
		{
			EvaluateInvCeresJacobians(lambda, x, r2, 1/factorInv, jacobians);
		}
	}

	Eigen::Vector2f applyInv(const Eigen::Vector2f &xd) const
	{
		Eigen::Vector2f res;
		EvaluateInvCeres(mLambda, xd, res, NULL);
		return res;
	}

protected:
	float mLambda;
	float mMaxRadiusSq; //Do not apply distortion after this radius. Keeps the distortion within the image limits where it was calibrated.

	//Calcualtes the jacobians in ceres format
	//jacobians[0][0:1] = [dxd/dlambda, dyd/dlambda]
	//jacobians[1][0:1] = [dxd/dx, dyd/dx]
	//jacobians[1][2:3] = [dxd/dy, dyd/dy]
	template <class TXMat>
	static void EvaluateCeresJacobians(double lambda, const Eigen::MatrixBase<TXMat> &x, double r2, double factor, double **jacobians)
	{
		const double factor2 = factor*factor;
		if (jacobians[0])
		{
			//dxd/dlambda
			jacobians[0][0] = -x[0] * r2*factor2;
			//dyd/dlambda
			jacobians[0][1] = -x[1] * r2*factor2;
		}
		if (jacobians[1])
		{
			//dxd/dx
			jacobians[1][0] = (1 + lambda*(x[1] * x[1] - x[0] * x[0]))*factor2;
			//dxd/dy
			jacobians[1][1] = -2 * lambda*x[0] * x[1] * factor2;

			//dyd/dx
			jacobians[1][2] = -2 * lambda*x[0] * x[1] * factor2;
			//dyd/dy
			jacobians[1][3] = (1 + lambda*(x[0] * x[0] - x[1] * x[1]))*factor2;
		}
	}

	//jacobians[0][0:1] = [dx/dlambda, dy/dlambda]
	//jacobians[1][0:1] = [dx/dxd, dy/dxd]
	//jacobians[1][2:3] = [dx/dyd, dy/dyd]
	template <class TXMat>
	static void EvaluateInvCeresJacobians(double lambda, const Eigen::MatrixBase<TXMat> &x, double r2, double factor, double **jacobians)
	{
		static_assert(TXMat::SizeAtCompileTime == 2, "X must be of size 2");
		assert(jacobians != NULL);

		//Compute forward
		double dxd_dLambda[2];
		double dxd_dx[4];
		double *jacPtr[2] = { dxd_dLambda, dxd_dx };

		EvaluateCeresJacobians(lambda, x, r2, factor, jacPtr);

		//Build square jacobian
		//Results = {lambda, xd, yd}, Params = {lambda, x, y}
		//jacForward(i,j) = dResult[i] / dParams[j]
		//jacForward(2,0) = dyd / dlambda
		//jacForward(2,1) = dyd / dx
		Eigen::Matrix3dr jacForward;
		jacForward(0, 0) = 1;
		jacForward(0, 1) = 0;
		jacForward(0, 2) = 0;

		jacForward(1, 0) = dxd_dLambda[0]; //dxd/dlambda
		jacForward(1, 1) = dxd_dx[0]; //dxd/dx
		jacForward(1, 2) = dxd_dx[1]; //dxd/dy

		jacForward(2, 0) = dxd_dLambda[1]; //dyd/dlambda
		jacForward(2, 1) = dxd_dx[2]; //dyd/dx
		jacForward(2, 2) = dxd_dx[3]; //dyd/dy

		//Invert
		//jacInv(i,j) = dParams[i] / dResult[j]
		Eigen::Matrix3dr jacInv;
		jacInv = jacForward.inverse();

		//Fill output
		if (jacobians[0])
		{
			//dx/dlambda
			jacobians[0][0] = jacInv(1, 0);
			//dy/dlambda
			jacobians[0][1] = jacInv(2, 0);
		}
		if (jacobians[1])
		{
			//dx/dxd
			jacobians[1][0] = jacInv(1, 1);
			//dx/dyd
			jacobians[1][1] = jacInv(1, 2);

			//dy/dxd
			jacobians[1][2] = jacInv(2, 1);
			//dy/dyd
			jacobians[1][3] = jacInv(2, 2);
		}
	}
};

}

#endif /* CAMERADISTORTIONMODEL_H_ */
