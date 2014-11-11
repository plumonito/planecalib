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
//#include "ReprojectionError3D.h"
//#include "EpipolarSegmentError.h"
#include "PoseTracker.h"

namespace planecalib
{


class ReprojectionError
{
public:
	ReprojectionError(const FeatureMeasurement &m):
		ReprojectionError(m.getOctave(), m.getPosition())
	{
	}

	ReprojectionError(const int octave, const Eigen::Vector2f &imgPoint):
		mScale(1<<octave), mImgPoint(imgPoint) 
	{
	}

	static const int kResidualCount = 2;

	template<class T>
	bool operator() (const T * const homography, const T * const x, T *residuals) const;

	void evalToErrors(const Eigen::Matrix3dr &homography, const Eigen::Vector2d &x, const float errorThreshold, MatchReprojectionErrors &errors) const;

protected:
	const int mScale;
	const Eigen::Vector2f mImgPoint;
};

template<class T>
bool ReprojectionError::operator () (const T * const homography, const T * const x, T *residuals) const
{
	T p[3];

	//Translate
	p[0] = homography[0] * x[0] + homography[1] * x[1] + homography[2];
	p[1] = homography[3] * x[0] + homography[4] * x[1] + homography[5];
	p[2] = homography[6] * x[0] + homography[7] * x[1] + homography[8];

	//Normalize
	p[0] /= p[2];
	p[1] /= p[2];

	//Residuals
	residuals[0] = (T(mImgPoint.x()) - p[0]) / T(mScale);
	residuals[1] = (T(mImgPoint.y()) - p[1]) / T(mScale);
	return true;
}

void ReprojectionError::evalToErrors(const Eigen::Matrix3dr &homography, const Eigen::Vector2d &x, const float errorThresholdSq, MatchReprojectionErrors &errors) const
{
	assert(kResidualCount == 2);
	
	double residuals[kResidualCount];
	this->operator()(homography.data(), x.data(), residuals);
	
	errors.reprojectionErrorsSq = (float)(residuals[0] * residuals[0] + residuals[1] * residuals[1]);
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
			if(feature.getMeasurements().size() > 1)
			{
				mFeaturesToAdjust.insert(&feature);
			}
		}
	}
}

bool BundleAdjuster::isInlier(const FeatureMeasurement &measurement, const Eigen::Matrix3dr &pose, const Eigen::Vector2d &position)
{
	MatchReprojectionErrors errors;

	ReprojectionError err(measurement);
	err.evalToErrors(pose, position, mOutlierPixelThresholdSq, errors);
	return errors.isInlier;
}

void BundleAdjuster::getInliers(const std::unordered_map<Keyframe *, Eigen::Matrix3dr> &paramsPoses,
		const std::unordered_map<Feature *, Eigen::Vector2d> &paramsFeatures,
		const std::vector<FeatureMeasurement> &measurements,
		int &inlierCount)
{
	inlierCount = 0;
	for(auto &m : measurements)
	{
		auto itFeature = paramsFeatures.find(&m.getFeature());
		auto itPose = paramsPoses.find(&m.getKeyframe());
		if(isInlier(m, itPose->second, itFeature->second))
			inlierCount++;
	}
}

/*
class BAIterationCallback : public ceres::IterationCallback
{
public:
	BAIterationCallback(const SlamRegion *region) :
		mRegion(region)
	{}

	virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
	{
		if (mRegion->getAbortBA())
			return ceres::SOLVER_ABORT;
		else
			return ceres::SOLVER_CONTINUE;
	}

protected:
	const SlamRegion *mRegion;
};
*/

BundleAdjuster::TGetPoseParamsResult BundleAdjuster::getPoseParams(Keyframe &frame, std::unordered_map<Keyframe *, Eigen::Matrix3dr> &paramsPoses)
{
	auto itNew = paramsPoses.emplace(&frame, Eigen::Matrix3dr());

	if(itNew.second)
	{
		//Copy pose params
		auto &pose = frame.getPose();

		auto &params = itNew.first->second;
		params = pose.cast<double>();
	}
	return itNew;
}

bool BundleAdjuster::bundleAdjust()
{
	ProfileSection s("bundleAdjust");

	if(mFramesToAdjust.empty())
		return true;

	//Gather relevant measurements
	std::unordered_map<Keyframe *, Eigen::Matrix3dr> paramsPoses;
	std::unordered_map<Feature *, Eigen::Vector2d> paramsFeatures;

	std::vector<FeatureMeasurement> measurements;

	//BA ceres problem
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.dense_linear_algebra_library_type = ceres::LAPACK;

	options.num_threads = 4;
	options.num_linear_solver_threads = 4;
	options.logging_type = ceres::SILENT;
	
	options.minimizer_progress_to_stdout = false;

    // in some environments ceres uses std::tr1::shared_ptr, in others
    // it uses std::shared_ptr. Let's keep it simple by not using
    // make_shared.
	options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering());

	//Abort callback
	//std::unique_ptr<BAIterationCallback> callback;
	//if (!mIsExpanderBA)
	//{
	//	callback.reset(new BAIterationCallback(mRegion));
	//	options.callbacks.push_back(callback.get());
	//}

	ceres::Problem problem;

	//Reset new key frame flag
	//if (!mIsExpanderBA)
	//	mRegion->setAbortBA(false);

	//Read-lock to prepare ceres problem
	{
		ProfileSection sconstruct("construct");

		shared_lock<shared_mutex> lockRead(mMap->getMutex(), std::defer_lock);
		if (mUseLocks)
			lockRead.lock();

		assert(mMap->getKeyframes().size() >= 2);
		assert(!mFramesToAdjust.empty());
		assert(!mFeaturesToAdjust.empty());

		//Prepare poses
		for (auto &framePtr : mFramesToAdjust)
		{
			Keyframe &frame = *framePtr;

			//Add frame to params list
			auto itNew = getPoseParams(frame, paramsPoses);
			auto &params = itNew.first->second;

			//Add pose as parameter block
			problem.AddParameterBlock(params.data(), 9);
			options.linear_solver_ordering->AddElementToGroup(params.data(), 1);

			if (&frame == mMap->getKeyframes().begin()->get())
			{
				//First key frame in region, pose fixed
				problem.SetParameterBlockConstant(params.data());
			}
		}

		//Prepare features
		for (auto &featurePtr : mFeaturesToAdjust)
		{
			Feature &feature = *featurePtr;

			//We need at least two measurements to bundle adjust
			if (feature.getMeasurements().size() > 1)
			{
				//Add feature to params list
				auto itNew = paramsFeatures.emplace(&feature, feature.getPosition().cast<double>());
				auto &params = itNew.first->second;

				//Add 3D feature as parameter block
				problem.AddParameterBlock(params.data(), 2);
				options.linear_solver_ordering->AddElementToGroup(params.data(), 0);

				//Measurements will be added later so that we don't need to find the params again
			}
		}

		//Add all 3D feautres to ceres problem
		for (auto &params : paramsFeatures)
		{
			Feature &feature = *params.first;
			auto &featureParams = params.second;

			//Add all measurements as residual blocks
			for (auto &mPtr : feature.getMeasurements())
			{
				FeatureMeasurement &m = *mPtr;
				Keyframe &frame = m.getKeyframe();

				//Make a copy of the measurement
				measurements.push_back(m);

				//Get pose
				auto itNewPose = getPoseParams(frame, paramsPoses);
				auto &poseParams = itNewPose.first->second;

				//Is this frame outside of bundle adjustment?
				if (itNewPose.second)
				{
					problem.AddParameterBlock(poseParams.data(), 9);
					options.linear_solver_ordering->AddElementToGroup(poseParams.data(), 1);
					problem.SetParameterBlockConstant(poseParams.data());
				}

				//const int scale = 1<<m.getOctave();
				//const double costScale = (feature.getMeasurements().size() > 3) ? 1e6 : 1.0;
				ceres::LossFunction *lossFunc_i = new ceres::CauchyLoss(mOutlierPixelThreshold);
				//ceres::LossFunction *scaledLoss = new ceres::ScaledLoss(lossFunc_i, costScale, ceres::TAKE_OWNERSHIP);

				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<ReprojectionError, ReprojectionError::kResidualCount, 9, 2>(
					new ReprojectionError(m)),
					lossFunc_i, poseParams.data(), featureParams.data());
			}
		}
	}

	//Get inliers before
	//int inlierCountBefore;
	//getInliers(paramsPoses, paramsFeatures3D, measurements3D, measurements2D, inlierCountBefore);
	//DTSLAM_LOG << "BA inlier count before: " << inlierCountBefore << "\n";

	//No locks while ceres runs
	//Non-linear minimization!
	ceres::Solver::Summary summary;
	{
		ProfileSection ssolve("solve");
		ceres::Solve(options, &problem, &summary);
	}

	MYAPP_LOG << "Uncalibrated BA report:\n" << summary.FullReport();

	//if (summary.termination_type == ceres::USER_FAILURE || (!mIsExpanderBA && mMap->getAbortBA()))
	//{
	//	DTSLAM_LOG << "\n\nBA aborted due to new key frame in map!!!\n\n";
	//	return false;
	//}
	if (summary.termination_type == ceres::FAILURE)
	{
		MYAPP_LOG << "\n\nBA solver failed!!!\n\n" << summary.FullReport();
		return false;
	}
	else
	{
		//Solver finished succesfully
		//Write-lock to update map
		{
			ProfileSection supdate("update");

			//std::unique_lock<std::mutex> lockLong(mMap->getLongOperationMutex(), std::defer_lock);
			//if(mUseLocks)
			//	lockLong.lock();

			std::unique_lock<shared_mutex> lockWrite(mMap->getMutex(), std::defer_lock);
			if(mUseLocks)
				lockWrite.lock();

			MYAPP_LOG << "BA updating map...\n";

			//Update all involved
			//Update frames
			for(auto &paramsPose : paramsPoses)
			{
				Keyframe &frame = *paramsPose.first;
				auto &params = paramsPose.second;
				frame.getPose() = params.cast<float>();
			}

			//Update features
			for(auto &params : paramsFeatures)
			{
				Feature &feature = *params.first;
				auto &featureParams = params.second;

				feature.setPosition(featureParams.cast<float>());

				//if (!mIsExpanderBA)
				//{
				//	//Update status
				//	int inlierCount = 0;
				//	for (auto &mPtr : feature.getMeasurements())
				//	{
				//		auto &m = *mPtr;
				//		auto &pose = m.getKeyFrame().getPose();

				//		MatchReprojectionErrors errors;
				//		BAReprojectionError3D err(m);
				//		err.evalToErrors(pose.getRotation(), pose.getTranslation(), feature.getPosition(), mOutlierPixelThreshold, errors);
				//		if (errors.isInlier)
				//			inlierCount++;
				//	}
				//	feature.setStatus(inlierCount);

				//	//Delete?
				//	if (inlierCount < (int)std::ceil(0.7f*feature.getMeasurements().size()))
				//	{
				//		feature.getRegion()->moveToGarbage(feature);
				//	}
				//}
			}

			MYAPP_LOG << "BA updating done.\n";

			//Mark that we performed BA
			if (mTracker)
				mTracker->resync();
		}

		//Get inliers after
		//int totalCount = measurements3D.size() + measurements2D.size();
		//int inlierCount;
		//getInliers(paramsPoses, paramsFeatures3D, measurements3D, measurements2D, inlierCount);
		//int inlierPercent = (int)(inlierCount*100.0f/totalCount);
		//DTSLAM_LOG << "BA inlier count after: " << inlierCount << " (" << inlierPercent << "%), time: " << summary.total_time_in_seconds << "s\n";

		return true;
	}
}

} /* namespace dtslam */
