
#include "HomographyCalibration.h"

#include <memory>
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "log.h"
#include "cvutils.h"

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

		T alphaInv2 = T(1.0) / (alpha*alpha);
		T u0alphaInv2 = u0*alphaInv2;
		T v0alphaInv2 = u0*alphaInv2;
		Eigen::Matrix<T, 3, 3> W;
		W <<
			alphaInv2, 0, -u0alphaInv2,  
			0,alphaInv2,-v0alphaInv2, 
			-u0alphaInv2,-v0alphaInv2,u0*u0alphaInv2+v0*v0alphaInv2+T(1.0);

		Eigen::Matrix<T, 3, 1> C1(
			-ny*u0, 
			alpha-ny*v0, 
			-ny);
		Eigen::Matrix<T, 3, 1> C2(
			nx*u0-alpha*(ny*ny+T(1.0)), 
			nx*v0+alpha*nx*ny, 
			nx);

		Eigen::Matrix<T, 3, 1> HC1;
		homographyPoint(C1, HC1);

		Eigen::Matrix<T, 3, 1> HC2;
		homographyPoint(C2, HC2);

		//T WHC2
		//Residuals
		residuals[0] = HC1.dot(W * HC2);
		residuals[1] = HC1.dot(W * HC1) - HC2.dot(W * HC2);
        return true;
    }

private:
	const Eigen::Matrix3d &mH;
};

Eigen::Matrix3fr HomographyCalibration::calibrate(const std::vector<Eigen::Matrix3fr> &H, const Eigen::Vector2i &imageSize)
{
	int hcount = H.size();

	//Adjust for principal point
	Eigen::Matrix3fr T, Ti;
	T << 1, 0, -imageSize.x() / 2, 0, 1, -imageSize.y()/2, 0, 0, 1;
	Ti << 1, 0, +imageSize.x() / 2, 0, 1, +imageSize.y()/2, 0, 0, 1;
	
	std::vector<Eigen::Matrix3d> Ht(hcount);
	for (int i = 0; i < hcount; i++)
		Ht[i] = (T * H[i] * Ti).cast<double>();

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
	Eigen::Vector2d normal(0, 0); //Assume reference camera is perependicular to plane. Last entry is always 1

	for (int i = 0; i<hcount; i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<HomographyCalibrationError, 2, 1,2,2>(
			new HomographyCalibrationError(Ht[i])),
			lossFunc, &alpha,pp.data(),normal.data());
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);
	MYAPP_LOG << summary.FullReport();

	//Build final K
	Eigen::Matrix3fr K;
	K << alpha, 0, pp.x(), 0, alpha, pp.y(), 0, 0, 1;
	return K;
}

} 
