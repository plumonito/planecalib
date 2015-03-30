
#ifndef HOMOGRAPHYCALIBRATION_H_
#define HOMOGRAPHYCALIBRATION_H_

#include "eutils.h"
#include "CeresUtils.h"

namespace planecalib {

class HomographyCalibration
{
public:
	HomographyCalibration()
	{
	}

	double getInitialAlpha() const { return mInitialAlpha; }
	const Eigen::Matrix3fr &getK() const { return mK; }
	const Eigen::Vector3d &getNormal() const { return mNormal; }

	void calibrate(const std::vector<Eigen::Matrix3fr> &H, const Eigen::Vector2i &imageSize);

protected:
	double mInitialAlpha;
	Eigen::Matrix3fr mK;
	Eigen::Vector3d mNormal;
};

//////////////////////////////////////////////////////////////////

class HomographyCalibrationError
{
public:
	HomographyCalibrationError(const Eigen::Matrix3d &H_)
		: mH(H_)
	{
	}

	static const int kResidualCount = 2;

	template<typename T>
	void homographyPoint(const Eigen::Matrix<T, 3, 1> &C, Eigen::Matrix<T, 3, 1> &HC) const
	{
		HC[0] = T(mH(0, 0))*C[0] + T(mH(0, 1))*C[1] + T(mH(0, 2))*C[2];
		HC[1] = T(mH(1, 0))*C[0] + T(mH(1, 1))*C[1] + T(mH(1, 2))*C[2];
		HC[2] = T(mH(2, 0))*C[0] + T(mH(2, 1))*C[1] + T(mH(2, 2))*C[2];
	}

	template<typename T>
	static void GetBasis(const T * const x, Eigen::Matrix<T, 3, 1> &basis1, Eigen::Matrix<T, 3, 1> &basis2)
	{
		const double kThreshold = 0.1;

		//basis1 = cross(x,C)
		//basis2 = cross(x,basis1)
		//Check that the point we use is not colinear with x
		const double x1val = CeresUtils::ToDouble(x[1]);
		const double x2val = CeresUtils::ToDouble(x[2]);
		if (x1val > kThreshold || x1val < -kThreshold || x2val > kThreshold || x2val < -kThreshold)
		{
			//Use C=[1,0,0]
			basis1[0] = T(0);
			basis1[1] = x[2];
			basis1[2] = -x[1];

			basis2[0] = -(x[1] * x[1] + x[2] * x[2]);
			basis2[1] = x[0] * x[1];
			basis2[2] = x[0] * x[2];
		}
		else
		{
			//Use C=[0,1,0]
			basis1[0] = -x[2];
			basis1[1] = T(0);
			basis1[2] = x[0];

			basis2[0] = x[0] * x[1];
			basis2[1] = -(x[0] * x[0] + x[2] * x[2]);
			basis2[2] = x[1] * x[2];
		}
		//basis1.normalize();
		//basis2.normalize();
	}

	//Homography is in row-major order
	template<typename T>
	bool operator()(const T * const alpha_, const T * const pp, const T * const normal, T *residuals) const
	{
		const T &alpha = *alpha_;
		const T &u0 = pp[0];
		const T &v0 = pp[1];

		Eigen::Matrix<T, 3, 3> K;
		K <<
			alpha, T(0.0), u0,
			T(0.0), alpha, v0,
			T(0.0), T(0.0), T(1.0);

		T alphaInv = T(1.0) / alpha;
		Eigen::Matrix<T, 3, 3> Kinv;
		Kinv <<
			alphaInv, T(0.0), -u0*alphaInv,
			T(0.0), alphaInv, -v0*alphaInv,
			T(0.0), T(0.0), T(1.0);

		Eigen::Matrix<T, 3, 1> C1;
		Eigen::Matrix<T, 3, 1> C2;

		GetBasis(normal, C1, C2);
		C1 = K*C1;
		C2 = K*C2;

		Eigen::Matrix<T, 3, 1> HC1;
		homographyPoint(C1, HC1);

		Eigen::Matrix<T, 3, 1> KiHC1;
		KiHC1 = Kinv*HC1;

		Eigen::Matrix<T, 3, 1> HC2;
		homographyPoint(C2, HC2);

		Eigen::Matrix<T, 3, 1> KiHC2;
		KiHC2 = Kinv*HC2;

		T norm1Sq = KiHC1.squaredNorm();
		T norm1 = sqrt(norm1Sq);
		T norm2Sq = KiHC2.squaredNorm();
		T norm2 = sqrt(norm2Sq);

		//Residuals
		residuals[0] = KiHC1.dot(KiHC2) / (norm1*norm2);
		residuals[1] = T(1) - norm2Sq / norm1Sq;
		return true;

	}

private:
	const Eigen::Matrix3d &mH;
};

}

#endif 
