
#include "HomographyCalibration.h"

#include <memory>
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "log.h"
#include "cvutils.h"
#include "CeresUtils.h"
#include "CeresParametrization.h"
#include "Profiler.h"

namespace planecalib {

	//////////////////////////////////////////////////////////////////
// HomographyCalibrationError

class HomographyCalibrationError
{
public:
	HomographyCalibrationError(const Eigen::Matrix3d &H_, bool useNormalized)
		: mH(H_), mUseNormalized(useNormalized)
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

		//basis1 = cross(C,x)
		//basis2 = cross(basis1,x)
		//Check that the point we use is not colinear with x
		const double x0val = CeresUtils::ToDouble(x[0]);
		const double x2val = CeresUtils::ToDouble(x[2]);
		if (x0val > kThreshold || x0val < -kThreshold || x2val > kThreshold || x2val < -kThreshold)
		{
			//Use C=[0,1,0]
			basis1[0] = x[2];
			basis1[1] = T(0);
			basis1[2] = -x[0];

			basis2[0] = -x[0] * x[1];
			basis2[1] = x[0] * x[0] + x[2] * x[2];
			basis2[2] = -x[1] * x[2];
		}
		else
		{
			//Use C=[1,0,0]
			basis1[0] = T(0);
			basis1[1] = -x[2];
			basis1[2] = x[1];

			basis2[0] = x[1] * x[1] + x[2] * x[2];
			basis2[1] = -x[0] * x[1];
			basis2[2] = -x[0] * x[2];
		}
		//basis1.normalize();
		//basis2.normalize();
	}

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
		if (mUseNormalized)
		{
			residuals[0] = KiHC1.dot(KiHC2) / (norm1*norm2);
			residuals[1] = T(1) - norm2Sq / norm1Sq;
		}
		else
		{
			residuals[0] = KiHC1.dot(KiHC2);
			residuals[1] = norm1Sq - norm2Sq;
		}

		return true;

	}

	bool operator()(const double alpha, const Eigen::Vector2d &pp, const Eigen::Vector3d &normal, Eigen::Vector2d &residuals) const
	{
		return (*this)(&alpha, pp.data(), normal.data(), residuals.data());
	}

private:
	const bool mUseNormalized;
	const Eigen::Matrix3d &mH;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HomographyCalibrationRansac
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HomographyCalibrationRansac::setData(const std::vector<Eigen::Matrix3d> *homographies)
{
	assert(homographies);

	mHomographies = homographies;
	this->mConstraintCount = mHomographies->size();
}

std::vector<double> HomographyCalibrationRansac::modelFromMinimalSet(const std::vector<int> &constraintIndices)
{
	Eigen::Matrix<double, 2, 1> A;
	Eigen::Matrix<double, 2, 1> b;

	//Hti must be column-major!!!
	auto &Hti = mHomographies->at(constraintIndices[0]);

	A[0] = -Hti(2)*Hti(5);
	b[0] = Hti(0)*Hti(3) + Hti(1)*Hti(4);
	A[1] = Hti(5)*Hti(5) - Hti(2)*Hti(2);
	b[1] = -(Hti(3)*Hti(3) - Hti(0)*Hti(0) + Hti(4)*Hti(4) - Hti(1)*Hti(1));
	
	std::vector<double> res;
	res.push_back(sqrt(A.dot(b) / (A.dot(A)))); //alpha = (pinv(A)*b)^0.5;
	return std::move(res);
}

void HomographyCalibrationRansac::getInliers(const double &model, int &inlierCount, float &errorSumSq, HomographyCalibrationIterationData &data)
{
	ceres::CauchyLoss robustLoss(mOutlierErrorThreshold);
	
	inlierCount = 0;
	errorSumSq = 0;

	for (int i = 0; i < mConstraintCount; i++)
	{
		auto &Hti = mHomographies->at(i);

		HomographyCalibrationError err(Hti, true);
		Eigen::Vector2d residuals;
		err(model, Eigen::Vector2d(0, 0), Eigen::Vector3d(0, 0, 1), residuals);

		double errorSq = residuals.squaredNorm();

		if (errorSq < mOutlierErrorThresholdSq)
		{
			inlierCount++;
		}

		//Apply robust function
		double robustError[3];
		robustLoss.Evaluate(errorSq, robustError);
		errorSumSq += (float)robustError[0];
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////

void HomographyCalibration::initFromCamera(const CameraModel &camera)
{
	mPrincipalPoint = camera.getPrincipalPoint().cast<double>();
	//mDistortionParams = camera.getDistortionModel().getParams();
	mFocalLengths = camera.getFocalLength().cast<double>();
}

void HomographyCalibration::updateCamera(CameraModel &camera) const
{
	camera.getPrincipalPoint() = mPrincipalPoint.cast<float>();
	//camera.getDistortionModel.setParams(mDistortionParams);
	camera.getFocalLength() = mFocalLengths.cast<float>();
}

void HomographyCalibration::calibrateLinear(const std::vector<Eigen::Matrix3d> &H)
{
	ProfileSection s("linear");

	int hcount = H.size();

	HomographyCalibrationRansac ransac;
	ransac.setParams(0.1, std::min(10, hcount), hcount, hcount);
	ransac.setData(&H);
	ransac.doRansac();
	mInitialAlpha = ransac.getBestModel();

	if (mVerbose)
	{
		MYAPP_LOG << "HomographyCalibrateRansac, inliers=" << ransac.getBestInlierCount() << "/" << hcount << "\n";
	}

	////Assume normal is [0,0,1] and find focal length
	//Eigen::VectorXd A(2 * hcount), b(2 * hcount);
	//for (int i = 0; i < hcount; i++)
	//{
	//	auto &Hti = H[i];

	//	//Note: Hti must be column-major!
	//	A[2 * i] = -Hti(2)*Hti(5);
	//	b[2 * i] = Hti(0)*Hti(3) + Hti(1)*Hti(4);
	//	A[2 * i + 1] = Hti(5)*Hti(5) - Hti(2)*Hti(2);
	//	b[2 * i + 1] = -(Hti(3)*Hti(3) - Hti(0)*Hti(0) + Hti(4)*Hti(4) - Hti(1)*Hti(1));
	//}

	//double alpha = sqrt(A.dot(b) / (A.dot(A))); //alpha = (pinv(A)*b)^0.5;
	//mInitialAlpha = alpha;
}

void HomographyCalibration::calibrateNonLinear(const std::vector<Eigen::Matrix3d> &H)
{
	ProfileSection s("nonlinear");

	int hcount = H.size();

	//Non-linear minimization
	ceres::Problem problem;
	ceres::LossFunction *lossFunc = NULL;
	//lossFunc = new ceres::CauchyLoss(3.0);

	Eigen::Vector2d pp(0, 0); //Homographies have been normalized so principal point is at the origin
	mNormal << 0, 0, 1; //Assume reference camera is perependicular to plane.

	double alpha = mInitialAlpha;
	problem.AddParameterBlock(&alpha, 1);
	problem.AddParameterBlock(pp.data(), 2);
	if (mFixPrincipalPoint)
		problem.SetParameterBlockConstant(pp.data());
	problem.AddParameterBlock(mNormal.data(), 3, new Fixed3DNormParametrization(1));
	//problem.SetParameterBlockConstant(mNormal.data());
	for (int i = 0; i<hcount; i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<HomographyCalibrationError, HomographyCalibrationError::kResidualCount, 1, 2, 3>(
			new HomographyCalibrationError(H[i], mUseNormalizedConstraints)),
			lossFunc, &alpha, pp.data(), mNormal.data());
	}

	ceres::Solver::Options options;
	//options.max_num_iterations = 200;
	//options.gradient_tolerance = 1e-25;
	//options.parameter_tolerance = 1e-25;
	//options.function_tolerance = 1e-25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;

	ceres::Solver::Summary summary;
	{
	ProfileSection s("solve");
	ceres::Solve(options, &problem, &summary);
	}
	//Update
	mPrincipalPoint += pp;
	mFocalLengths[0] = mFocalLengths[1] = alpha;

	//Show
	if (mVerbose)
	{
		MYAPP_LOG << "Homography calib results\n";
		MYAPP_LOG << "Use normalized constraints: " << mUseNormalizedConstraints << "\n";
		MYAPP_LOG << summary.FullReport();

		MYAPP_LOG << "Initial alpha=" << mInitialAlpha << ", final=" << alpha << "\n";

		MYAPP_LOG << "PP=" << mPrincipalPoint.transpose() << "\n";
		MYAPP_LOG << "Normal=" << mNormal.transpose() << "\n";
	}
}

void HomographyCalibration::calibrate(const std::vector<Eigen::Matrix3fr> &H)
{
	ProfileSection s("HomographyCalibrate");

	int hcount = H.size();
	std::vector<Eigen::Matrix3d> Ht(hcount);
	for (int i = 0; i < hcount; i++)
	{
		Ht[i] = H[i].cast<double>();
		//Ht[i] = (T * H[i] * Ti).cast<double>();
		//Ht[i] /= Ht[i].determinant();
	}

	calibrateLinear(Ht);
	calibrateNonLinear(Ht);
}

} 
