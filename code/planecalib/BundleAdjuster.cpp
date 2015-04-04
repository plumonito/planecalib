/*
 * BundleAdjuster.cpp
 *
 *  Created on: 6.3.2014
 *      Author: dan
 */

#include "BundleAdjuster.h"
#include "gflags/gflags.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <opencv2/calib3d.hpp>
#include <ceres/ceres.h>
#include <mutex>
#include "shared_mutex.h"
#include "Profiler.h"
#include "CeresParametrization.h"
#include "PoseTracker.h"
#include "CameraDistortionModel.h"

namespace planecalib
{


class ReprojectionError
{
public:
	ReprojectionError(const Eigen::Vector2i &imageSize, const FeatureMeasurement &m):
		ReprojectionError(imageSize, m.getOctave(), m.getPosition())
	{
	}

	ReprojectionError(const Eigen::Vector2i &imageSize, const int octave, const Eigen::Vector2f &imgPoint) :
		mMaxRadiusSq(RadialCameraDistortionModel::MaxRadiusSqFromImageSize(imageSize)), mScale(1<<octave), mImgPoint(imgPoint) 
	{
	}

	static const int kResidualCount = 2;

	template<class T>
	bool operator() (const T * const _distortion, const T * const _homography, const T * const _x, T *residuals) const;

	void evalToErrors(const Eigen::Vector2d &distortion, const Eigen::Matrix3dr &homography, const Eigen::Vector2d &x, const float errorThreshold, MatchReprojectionErrors &errors) const;

protected:
	const double mMaxRadiusSq;
	const int mScale;
	const Eigen::Vector2f mImgPoint;
};

template<typename Derived>
struct ble
{
	typedef Eigen::MatrixBase<Derived> type;
};

template<class T>
bool ReprojectionError::operator () (const T * const _distortion, const T * const _homography, const T * const _x, T *residuals) const
{
	Eigen::Map<Eigen::Matrix<T, 2, 1>> distortion((T*)_distortion);
	Eigen::Map<Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> homography((T*)_homography);
	Eigen::Map<Eigen::Matrix<T, 2, 1>> x((T*)_x);


	//Homography
	const Eigen::Matrix<T, 3, 1> x3(x[0], x[1], T(1));
	const Eigen::Matrix<T, 3, 1> p = homography*x3;
	//p[0] = homography[0] * x[0] + homography[1] * x[1] + homography[2];
	//p[1] = homography[3] * x[0] + homography[4] * x[1] + homography[5];
	//p[2] = homography[6] * x[0] + homography[7] * x[1] + homography[8];

	//Normalize
	const Eigen::Matrix<T, 2, 1> pn = p.hnormalized();

	//Distort
	Eigen::Matrix<T, 2, 1> pd;
	RadialCameraDistortionModel::DistortPoint(mMaxRadiusSq, distortion, pn, pd);

	//Residuals
	residuals[0] = (T(mImgPoint.x()) - pd[0]) / T(mScale);
	residuals[1] = (T(mImgPoint.y()) - pd[1]) / T(mScale);
	return true;
}

void ReprojectionError::evalToErrors(const Eigen::Vector2d &distortion, const Eigen::Matrix3dr &homography, const Eigen::Vector2d &x, const float errorThresholdSq, MatchReprojectionErrors &errors) const
{
	assert(kResidualCount == 2);
	
	Eigen::Vector2d residuals;
	this->operator()(distortion.data(), homography.data(), x.data(), residuals.data());
	
	errors.reprojectionErrorsSq = (float)residuals.squaredNorm();
	errors.isInlier = errors.reprojectionErrorsSq < errorThresholdSq;
}




void BundleAdjuster::addFrameToAdjust(Keyframe &newFrame)
{
	//Add
	auto itNewFrame=mFramesToAdjust.insert(&newFrame);
	if(itNewFrame.second)
	{
		//New frame!

		//Add features too
		for(auto itM=newFrame.getMeasurements().begin(),endM=newFrame.getMeasurements().end(); itM!=endM; ++itM)
		{
			Feature &feature = (*itM)->getFeature();

			//We need at least two measurements to bundle adjust
			//if(feature.getMeasurements().size() > 1)
			{
				mFeaturesToAdjust.insert(&feature);
			}
		}
	}
}

bool BundleAdjuster::isInlier(const FeatureMeasurement &measurement, const Eigen::Matrix3dr &pose, const Eigen::Vector2d &position)
{
	MatchReprojectionErrors errors;

	ReprojectionError err(mImageSize, measurement);
	err.evalToErrors(mParamsDistortion, pose, position, mOutlierPixelThresholdSq, errors);
	return errors.isInlier;
}

void BundleAdjuster::getInliers(int &inlierCount)
{
	inlierCount = 0;
	for(auto &mp : mMeasurementsInProblem)
	{
		auto &m = *mp;
		if(isInlier(m, getPoseParams(&m.getKeyframe()), getFeatureParams(&m.getFeature())))
			inlierCount++;
	}
}

Eigen::Matrix3dr &BundleAdjuster::getPoseParams(Keyframe *framep)
{
	auto &frame = *framep;
	auto itNew = mParamsPoses.emplace(&frame, Eigen::Matrix3dr());
	auto &params = itNew.first->second;
	if (itNew.second)
	{
		//Is new, create
		params = frame.getPose().cast<double>();
	}

	return params;
}

Eigen::Vector2d &BundleAdjuster::getFeatureParams(Feature *featurep)
{
	auto &feature = *featurep;
	auto itNew = mParamsFeatures.emplace(&feature, Eigen::Vector2d());
	auto &params = itNew.first->second;
	if (itNew.second)
	{
		//Is new, create
		params = feature.getPosition().cast<double>();
	}

	return params;
}


bool BundleAdjuster::bundleAdjust()
{
	ProfileSection s("bundleAdjust");

	if (mFramesToAdjust.empty())
		return true;

	//BA ceres problem
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::CGNR;
	//options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.dense_linear_algebra_library_type = ceres::LAPACK;

	options.max_num_iterations = 500;
	options.num_threads = 4;
	options.num_linear_solver_threads = 4;
	options.logging_type = ceres::SILENT;

	options.minimizer_progress_to_stdout = false;

	//options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering());

	ceres::Problem problem;

	//Read-lock to prepare ceres problem
	{
		ProfileSection sconstruct("construct");

		shared_lock<shared_mutex> lockRead(mMap->getMutex(), std::defer_lock);
		if (mUseLocks)
			lockRead.lock();

		//assert(mMap->getKeyframes().size() >= 2);
		assert(!mFramesToAdjust.empty());
		assert(!mFeaturesToAdjust.empty());

		//Prepare poses
		for (auto &framep : mFramesToAdjust)
		{
			auto &frame = *framep;

			//Create and init params
			auto &params = getPoseParams(&frame);

			//Add pose as parameter block
			if (&frame == mMap->getKeyframes().begin()->get())
			{
				//First key frame in region, scale fixed
				problem.AddParameterBlock(params.data(), 9);
				//problem.AddParameterBlock(params.data() + 3, 3, new Fixed3DNormParametrization(1));
				problem.SetParameterBlockConstant(params.data());
			}
			else
			{
				problem.AddParameterBlock(params.data(), 9);
			}
			//options.linear_solver_ordering->AddElementToGroup(params.data(), 1);
			//options.linear_solver_ordering->AddElementToGroup(params.data()+3, 1);
		}

		//Prepare features
		for (auto &featurep : mFeaturesToAdjust)
		{
			auto &feature = *featurep;

			//Create and init params
			auto &params = getFeatureParams(&feature);

			//Add feature as parameter block
			problem.AddParameterBlock(params.data(), 2);
			//problem.SetParameterBlockConstant(params.data());
			//options.linear_solver_ordering->AddElementToGroup(params.data(), 0);
		}

		//Distortion params
		mImageSize = (**mFramesToAdjust.begin()).getImageSize();
		mParamsDistortion = Eigen::Vector2d::Zero();
		problem.AddParameterBlock(mParamsDistortion.data(), mParamsDistortion.rows());

		//Gather measurements
		for (auto &featurep : mFeaturesToAdjust)
		{
			auto &feature = *featurep;

			//if (feature.getMeasurements().size() <= 1)
			//	continue; //Skip if only one measurement

			//Add all measurements as residual blocks
			for (auto &mPtr : feature.getMeasurements())
			{
				FeatureMeasurement &m = *mPtr;
				mMeasurementsInProblem.push_back(&m);
			}
		}


		//Add measurements to problem
		for (auto mp : mMeasurementsInProblem)
		{
			auto &m = *mp;

			Feature &feature = m.getFeature();
			auto &featureParams = getFeatureParams(&feature);

			Keyframe &frame = m.getKeyframe();
			auto &poseParams = getPoseParams(&frame);

			//Is this frame outside of bundle adjustment?
			if (mFramesToAdjust.find(&frame) == mFramesToAdjust.end())
			{
				problem.AddParameterBlock(poseParams.data(), 9);
				problem.SetParameterBlockConstant(poseParams.data());
				//options.linear_solver_ordering->AddElementToGroup(poseParams.data(), 1);
			}

			ceres::LossFunction *lossFunc_i = NULL;
			//lossFunc_i = new ceres::CauchyLoss(mOutlierPixelThreshold);

			problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<ReprojectionError, ReprojectionError::kResidualCount, 2, 9, 2>(
				new ReprojectionError(mImageSize, m)),
				lossFunc_i, mParamsDistortion.data(), poseParams.data(), featureParams.data());
		}
	}

	//Get inliers before
	int inlierCount;
	getInliers(inlierCount);
	MYAPP_LOG << "BA inlier count: " << inlierCount << "\n";

	//No locks while ceres runs
	//Non-linear minimization!
	ceres::Solver::Summary summary;
	{
		ProfileSection ssolve("solve");
		ceres::Solve(options, &problem, &summary);
	}

	MYAPP_LOG << "BA report:\n" << summary.FullReport();
	MYAPP_LOG << "Distortion coefficients: " << mParamsDistortion.transpose() << "\n";
	//if (summary.termination_type == ceres::USER_FAILURE || (!mIsExpanderBA && mMap->getAbortBA()))
	//{
	//	DTSLAM_LOG << "\n\nBA aborted due to new key frame in map!!!\n\n";
	//	return false;
	//}
	if (summary.termination_type == ceres::FAILURE)
	{
		MYAPP_LOG << "\n\nBA solver failed!!!\n\n"; //<< summary.FullReport();
		return false;
	}

	getInliers(inlierCount);
	MYAPP_LOG << "BA inlier count: " << inlierCount << "\n";

	//Update pose
	for (auto &p : mParamsPoses)
	{
		auto &frame = *p.first;
		auto &params = p.second;

		frame.setPose(params.cast<float>());
	}

	//Update positions
	for (auto &p : mParamsFeatures)
	{
		auto &feature = *p.first;
		auto &params = p.second;

		feature.setPosition(params.cast<float>());
	}
	return true;
}

} /* namespace dtslam */
