#include "CameraDistortionModel.h"
#include "log.h"

namespace planecalib {

Eigen::Vector2f RadialCameraDistortionModel::undistortPoint(const Eigen::Vector2f &pd) const
{
		const int kMaxIters = 11;
		Eigen::Vector2f pn = pd;
	    for( int j = 0; j < kMaxIters; j++ )
	    {
	    	float r2 = pn.squaredNorm();
			float icdist = 1 + r2*mCoefficients[0] + r2*r2*mCoefficients[1];
			pn = (1 / icdist)*pd;
	    }

	return pn;
}

//class CreateInverseError
//{
//public:
//	CreateInverseError(double in, double out):
//		mRadiusIn(in), mRadiusOut(out)
//	{}
//
//	template<class T>
//	bool operator()(const T * const params, T*residuals) const
//	{
//		T r = T(mRadiusIn);
//		T r2 = r*r;
//		T r4 = r2*r2;
//		T outExp = r*(T(1) + params[0] * r2 + params[1] * r4);
//		residuals[0] = outExp - T(mRadiusOut);
//		return true;
//	}
//
//protected:
//	double mRadiusIn;
//	double mRadiusOut;
//};
//
//RadialCameraDistortionModel RadialCameraDistortionModel::createInverseModel() const
//{
// Note: this doesn't seem to work. With artificial distoritons (e.g. [0.05,0]) it doesn't find an appropriate inverse model
//	Eigen::Vector2d params = Eigen::Vector2d::Zero();
//
//	//BA ceres problem
//	ceres::Solver::Options options;
//	options.linear_solver_type = ceres::CGNR;
//	options.dense_linear_algebra_library_type = ceres::LAPACK;
//
//	options.max_num_iterations = 500;
//	options.num_threads = 4;
//	options.num_linear_solver_threads = 4;
//	options.logging_type = ceres::SILENT;
//	options.minimizer_progress_to_stdout = false;
//
//	ceres::Problem problem;
//	problem.AddParameterBlock(params.data(), 2);
//	
//	const int kSamples = 200;
//	double maxR = std::sqrt(mMaxRadiusSq);
//	for (double r = 0; r <= maxR; r += maxR/kSamples)
//	{
//		const double r2 = r*r;
//		const double r4 = r2*r2;
//		const double distortedR = r*(1.0 + mCoefficients[0] * r2 + mCoefficients[1] * r4);
//
//		problem.AddResidualBlock(
//			new ceres::AutoDiffCostFunction<CreateInverseError, 1, 2>(
//			new CreateInverseError(distortedR, r)),
//			NULL, params.data());
//	}
//
//	//Solve
//	ceres::Solver::Summary summary;
//	ceres::Solve(options, &problem, &summary);
//	MYAPP_LOG << "Create inv distortion report:\n" << summary.FullReport();
//	MYAPP_LOG << "Forward: " << mCoefficients.transpose() << "\n";
//	MYAPP_LOG << "Inverse: " << params.transpose() << "\n";
//
//	//Return
//	RadialCameraDistortionModel invModel;
//	invModel.init(params.cast<float>(), mMaxRadiusSq);
//
//	return invModel;
//}

Eigen::Vector2f DivisionDistortionModel::applyInv(const Eigen::Vector2f &pd) const
{
	const int kMaxIters = 11;
	Eigen::Vector2f pn = pd;
	for (int j = 0; j < kMaxIters; j++)
	{
		float r2 = pn.squaredNorm();
		float icdist = 1 + r2*mLambda;
		pn = icdist*pd;
	}

	return pn;
}

}
