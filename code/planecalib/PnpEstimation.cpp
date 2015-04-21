#include "PnpEstimation.h"
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include "CeresParametrization.h"
#include "CeresUtils.h"
#include "ReprojectionError3D.h"

#include "flags.h"

namespace planecalib {

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PnPRansac
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PnPRansac::PnPRansac()
{
}

PnPRansac::~PnPRansac()
{
}

void PnPRansac::setData(const std::vector<FeatureMatch> &matches, const CameraModel *camera)
{
	assert(matches.size()!=0);

	mMatchCount = matches.size();
	
	mOwnRefPoints.reset(new std::vector<Eigen::Vector3f>());
	mOwnImgPoints.reset(new std::vector<Eigen::Vector2f>());

	mRefPoints = mOwnRefPoints.get();
	mImgPoints = mOwnImgPoints.get();

	mConstraintCount = mMatchCount;

	//Normalize
	//TODO: this will break with fish-eye lenses, but cv::triangulate and cv:solvePnP can only take 2D points
	mOwnRefPoints->resize(mMatchCount);
	mOwnImgPoints->resize(mMatchCount);
	for (int i = 0; i != mMatchCount; ++i)
	{
		const FeatureMatch &match = matches[i];
		
		mOwnRefPoints->at(i) = match.getFeature().mPosition3D;
		mOwnImgPoints->at(i) = camera->unprojectToWorld(match.getPosition()).hnormalized().eval();

		//Create error functor
		mErrorFunctors.emplace_back(new PoseReprojectionError3D(camera, match));
	}
}

void PnPRansac::setData(const std::vector<Eigen::Vector3f> *refPoints, const std::vector<Eigen::Vector2f> *imgPoints, const CameraModel *camera)
{
	assert(refPoints != NULL && imgPoints != NULL);
	assert(refPoints->size() != 0);
	assert(refPoints->size() == imgPoints->size());

	mMatchCount = refPoints->size();

	mOwnImgPoints.reset(new std::vector<Eigen::Vector2f>());

	mRefPoints = refPoints;
	mImgPoints = mOwnImgPoints.get();

	mConstraintCount = mMatchCount;

	//Normalize
	//TODO: this will break with fish-eye lenses, but cv::triangulate and cv:solvePnP can only take 2D points
	mOwnImgPoints->resize(mMatchCount);
	for (int i = 0; i != mMatchCount; ++i)
	{
		mOwnImgPoints->at(i) = camera->unprojectToWorld(imgPoints->at(i)).hnormalized().eval();

		//Create error functor
		mErrorFunctors.emplace_back(new PoseReprojectionError3D(camera, refPoints->at(i).cast<double>(), 0, imgPoints->at(i).cast<double>()));
	}
}

std::vector<std::pair<Eigen::Matrix3dr, Eigen::Vector3d>> PnPRansac::modelFromMinimalSet(const std::vector<int> &constraintIndices)
{
	assert(constraintIndices.size()==4);

	std::vector<cv::Point3f> refp(4);
	std::vector<cv::Point2f> imgp(4);
	for(int i=0; i<4; ++i)
	{
		const int idx = constraintIndices[i];
		refp[i] = eutils::ToCVPoint( mRefPoints->at(idx) );
		imgp[i] = eutils::ToCVPoint( mImgPoints->at(idx) );
	}

	std::vector<std::pair<Eigen::Matrix3dr, Eigen::Vector3d>> solutions;

	cv::Vec3d rvec,tvec;
	if(cv::solvePnP(refp, imgp, cv::Matx33f::eye(), cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_P3P))
	{
		cv::Matx33d R;
		cv::Rodrigues(rvec, R);

		Eigen::Matrix3dr eR = eutils::FromCV(R);
		Eigen::Vector3d eT = eutils::FromCV(tvec);;

		solutions.push_back(std::make_pair(eR, eT));
	}
    return std::move(solutions);
}

void PnPRansac::getInliers(const std::pair<Eigen::Matrix3dr, Eigen::Vector3d> &model, int &inlierCount, float &errorSumSq, PnPIterationData &data)
{
	inlierCount = 0;
	errorSumSq = 0;

	ceres::CauchyLoss robustLoss(mOutlierErrorThreshold);

	data.reprojectionErrors.resize(mMatchCount);

	for (int i = 0; i<mMatchCount; ++i)
	{
		auto &errorFunctor = *mErrorFunctors[i];
		auto &errors = data.reprojectionErrors[i];

		errors.reprojectionErrorsSq = (float)errorFunctor.evalToDistanceSq(model.first, model.second);
		errors.isInlier = (errors.reprojectionErrorsSq < mOutlierErrorThresholdSq);
			

		if(errors.isInlier)
			inlierCount++;

		double robustError[3];
		robustLoss.Evaluate(errors.reprojectionErrorsSq, robustError);
		errorSumSq += (float)robustError[0];
	}
}

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
