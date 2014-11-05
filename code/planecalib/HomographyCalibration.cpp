
#include "HomographyCalibration.h"

#include <memory>
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "log.h"
#include "cvutils.h"
#include "CeresParametrization.h"

namespace planecalib {

class HomographyCalibrationError
{
public:
	HomographyCalibrationError(const Eigen::Matrix3d &H_)
		: mH(H_)
    {
    }

	template<typename T>
	void homographyPoint(const Eigen::Matrix<T, 3, 1> &C, Eigen::Matrix<T, 3, 1> &HC) const
	{
		HC[0] = T(mH(0, 0))*C[0] + T(mH(0, 1))*C[1] + T(mH(0, 2))*C[2];
		HC[1] = T(mH(1, 0))*C[0] + T(mH(1, 1))*C[1] + T(mH(1, 2))*C[2];
		HC[2] = T(mH(2, 0))*C[0] + T(mH(2, 1))*C[1] + T(mH(2, 2))*C[2];
	}

	//Homography is in row-major order
    template<typename T>
	bool operator()(const T * const alpha_, const T * const pp, const T * const normal, T *residuals) const
    {
		const T &alpha = *alpha_;
		const T &u0 = pp[0];
		const T &v0 = pp[1];
		const T &nx = normal[0];
		const T &ny = normal[1];
		const T &nz = normal[2];
		Eigen::Matrix<T, 3, 3> skewN;
		skewN << T(0), -nz, ny, nz, T(0), -nx, -ny, nx, T(0);
		Eigen::Matrix<T, 3, 3> skewNsq = skewN*skewN;

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

		for (int k = 0; k < 3; k++)
		{
			Eigen::Matrix<T, 3, 1> C1;
			Eigen::Matrix<T, 3, 1> C2;
			//if (k == 0)
			//{
			//	C1 <<
			//		-ny*u0,
			//		alpha*nz - ny*v0,
			//		-ny;
			//	C2 <<
			//		nx*nz*u0 - alpha*(ny*ny + nz*nz),
			//		nx*nz*v0 + alpha*nx*ny,
			//		nx*nz;
			//}
			//else if (k==1)
			//{
			//	C1 <<
			//		nx*u0 - alpha*nz,
			//		nx*v0,
			//		nx;
			//	C2 <<
			//		ny*nz*u0 + alpha*nx*ny,
			//		ny*nz*v0 - alpha*(nx*nx + nz*nz),
			//		ny*nz;
			//}
			//else
			//{
			//	C1 <<
			//		alpha*ny,
			//		-alpha*nx,
			//		0;
			//	C2 <<
			//		alpha*nx*nz - u0*(nx*nx+ny*ny),
			//		alpha*ny*nz - v0*(nx*nx + ny*ny),
			//		-(nx*nx + ny*ny);
			//}
			
			Eigen::Matrix<T, 3, 1> ee(T(0),T(0),T(0));
			ee[k] = T(1.0);

			C1 = K*skewN*ee;
			C2 = K*skewNsq*ee;

			Eigen::Matrix<T, 3, 1> HC1;
			homographyPoint(C1, HC1);
			//HC1 /= HC1.norm();

			Eigen::Matrix<T, 3, 1> KiHC1;
			KiHC1 = Kinv*HC1;

			Eigen::Matrix<T, 3, 1> HC2;
			homographyPoint(C2, HC2);
			//HC2 /= HC2.norm();

			Eigen::Matrix<T, 3, 1> KiHC2;
			KiHC2 = Kinv*HC2;

			//T WHC2
			//Residuals
			residuals[2*k+0] = KiHC1.dot(KiHC2);
			residuals[2*k+1] = KiHC1.dot(KiHC1) - KiHC2.dot(KiHC2);
		}
        return true;
    }

private:
	const Eigen::Matrix3d &mH;
};

void HomographyCalibration::calibrate(const std::vector<Eigen::Matrix3fr> &H, const Eigen::Vector2i &imageSize)
{
	int hcount = H.size();

	//Adjust for principal point
	Eigen::Matrix3fr T, Ti;
	T << 1, 0, -imageSize.x() / 2, 0, 1, -imageSize.y()/2, 0, 0, 1;
	Ti << 1, 0, +imageSize.x() / 2, 0, 1, +imageSize.y()/2, 0, 0, 1;
	
	std::vector<Eigen::Matrix3d> Ht(hcount);
	for (int i = 0; i < hcount; i++)
	{
		Ht[i] = (T * H[i] * Ti).cast<double>();
		//Ht[i] /= Ht[i].norm();
	}

	//Assume normal is [0,0,1] and find focal length
	Eigen::VectorXd A(2 * hcount), b(2 * hcount);
	for (int i = 0; i < hcount; i++)
	{
		auto &Hti = Ht[i];

		//Note: Hti must be column-major!
		A[2 * i] = -Hti(2)*Hti(5);
		b[2 * i] = Hti(0)*Hti(3) + Hti(1)*Hti(4);
		A[2 * i + 1] = Hti(5)*Hti(5) - Hti(2)*Hti(2);
		b[2 * i + 1] = -(Hti(3)*Hti(3) - Hti(0)*Hti(0) + Hti(4)*Hti(4) - Hti(1)*Hti(1));
	}

	double alpha = sqrt(A.dot(b) / (A.dot(A))); //alpha = (pinv(A)*b)^0.5;
	MYAPP_LOG << "Initial alpha=" << alpha << "\n";

	//Non-linear minimization
	ceres::Problem problem;
	ceres::LossFunction *lossFunc = new ceres::CauchyLoss(3.0);

	Eigen::Vector2d pp(0,0); //Assume principal point is in the middle
	mNormal << 0, 0, 1; //Assume reference camera is perependicular to plane.

	problem.AddParameterBlock(&alpha, 1);
	problem.AddParameterBlock(pp.data(), 2);
	//problem.SetParameterBlockConstant(pp.data());
	problem.AddParameterBlock(mNormal.data(), 3, new Fixed3DNormParametrization(1));
	//problem.SetParameterBlockConstant(mNormal.data());
	for (int i = 0; i<hcount; i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<HomographyCalibrationError, 6, 1,2,3>(
			new HomographyCalibrationError(Ht[i])),
			lossFunc, &alpha,pp.data(),mNormal.data());
	}

	ceres::Solver::Options options;
	options.gradient_tolerance = 1e-25;
	options.parameter_tolerance = 1e-25;
	options.function_tolerance = 1e-25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	ceres::Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);
	MYAPP_LOG << summary.FullReport();

	//Build final K
	mK << alpha, 0, pp.x(), 0, alpha, pp.y(), 0, 0, 1;
}

} 
