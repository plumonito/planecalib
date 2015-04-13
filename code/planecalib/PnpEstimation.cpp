#include "PnpEstimation.h"
#include <ceres/ceres.h>
#include "CeresParametrization.h"
#include "CeresUtils.h"
#include "ReprojectionError3D.h"

#include "flags.h"

namespace planecalib {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PnPRefiner
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PnPRefiner::getReprojectionErrors(const FeatureMatch &match,
		const Eigen::Matrix3dr &R,
		const Eigen::Vector3d &translation,
		MatchReprojectionErrors &errors)
{
	PoseReprojectionError3D err(mCamera, match);
	errors.reprojectionErrorsSq = (float)err.evalToDistanceSq(R, translation);
	errors.isInlier = errors.reprojectionErrorsSq < mOutlierPixelThresholdSq;
	return errors.isInlier;
}


void PnPRefiner::getInliers(const std::vector<FeatureMatch> &matches,
		const Eigen::Matrix3dr &R,
		const Eigen::Vector3d &translation,
		int &inlierCount,
		std::vector<MatchReprojectionErrors> &errors)
{

	inlierCount = 0;

	const int matchCount=matches.size();
	errors.resize(matchCount);
	for(int i=0; i<matchCount; i++)
	{
		auto &match = matches[i];
		auto &error = errors[i];

		getReprojectionErrors(match, R, translation, error);
		
		if(error.isInlier)
			inlierCount++;
	}
}

void PnPRefiner::refinePose(const std::vector<FeatureMatch> &matches,
		Eigen::Matrix3fr &rotation,
		Eigen::Vector3f &translation,
		int &inlierCount,
		std::vector<MatchReprojectionErrors> &errors)
{
	const int matchCount=matches.size();

	////////////////////////////////////////////////////
	//Convert input to double

	//Rotation
	const Eigen::Matrix3dr rotation_d = rotation.cast<double>();
	Eigen::Vector3d rparams_d;
	ceres::RotationMatrixToAngleAxis(ceres::RowMajorAdapter3x3(rotation_d.data()), rparams_d.data());

	//Transation
	Eigen::Vector3d translation_d = translation.cast<double>();

	////////////////////////////////////////////////////
	//Check inliers before
	//int inlierCountBefore;
	//getInliers(matches, rparams_d, translation_d, inlierCountBefore, errors);
	//DTSLAM_LOG << "PnPRefine: inliers before: " << inlierCountBefore << "\n";

	int totalMatchCount=0;
	//int refineMatchCount=0;

	////////////////////////////////////////////////////
	// Prepare ceres problem
	//Non-linear minimization with distortion
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.dense_linear_algebra_library_type = ceres::LAPACK;

	options.num_threads = 1; //Multi-threading here adds too much overhead
	options.num_linear_solver_threads = 1;

	options.logging_type = ceres::SILENT;
	options.minimizer_progress_to_stdout = false;

	ceres::Problem problem;

	problem.AddParameterBlock(rparams_d.data(), 3);
	problem.AddParameterBlock(translation_d.data(), 3);

	for(int i=0; i<matchCount; i++)
	{
		const FeatureMatch &match = matches[i];
		auto &feature = match.getFeature();

		totalMatchCount++;

		ceres::LossFunction *lossFunc = new ceres::CauchyLoss(mOutlierPixelThreshold);

		problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<PoseReprojectionError3D,2,3,3>(
						new PoseReprojectionError3D(mCamera,match)),
						lossFunc, rparams_d.data(), translation_d.data());
	}

	
	////////////////////////////////////////////////////
	// Solve
	ceres::Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	//DTSLAM_LOG << summary.FullReport();

	////////////////////////////////////////////////////
	//Extract result
	Eigen::Matrix3dr finalRotation_d;
	ceres::AngleAxisToRotationMatrix(rparams_d.data(), ceres::RowMajorAdapter3x3(finalRotation_d.data()));
	rotation = finalRotation_d.cast<float>();

	translation = translation_d.cast<float>();

	getInliers(matches, finalRotation_d, translation_d, inlierCount, errors);
	MYAPP_LOG << "PnPRefine: final inlier count=" << inlierCount << "\n";
}

} /* namespace dtslam */
