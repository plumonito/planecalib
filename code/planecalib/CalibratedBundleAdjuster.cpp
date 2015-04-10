
#include "CalibratedBundleAdjuster.h"
#include "gflags/gflags.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <opencv2/calib3d.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <mutex>
#include "shared_mutex.h"
#include "Profiler.h"
#include "CeresParametrization.h"
//#include "ReprojectionError3D.h"
#include "CameraDistortionModel.h"
#include "PoseTracker.h"

namespace planecalib
{


class CalibratedReprojectionError
{
public:
	CalibratedReprojectionError(const Eigen::Vector2i &imageSize, const FeatureMeasurement &m) :
		CalibratedReprojectionError(imageSize, m.getOctave(), m.getPosition())
	{
	}

	CalibratedReprojectionError(const Eigen::Vector2i &imageSize, const int octave, const Eigen::Vector2f &imgPoint) :
		mMaxRadiusSq(RadialCameraDistortionModel::MaxRadiusSqFromImageSize(imageSize)), mScale(1 << octave), mImgPoint(imgPoint)
	{
	}

	static const int kResidualCount = 2;

	template<class T>
	bool operator() (const T * const _distortion, const T * const kparams, const T * const rparams, const T * const tparams, const T * const x, T *residuals) const;

protected:
	const double mMaxRadiusSq;
	const int mScale;
	const Eigen::Vector2f mImgPoint;
};

template<class T>
bool CalibratedReprojectionError::operator () (const T * const _distortion, const T * const kparams, const T * const rparams, const T * const _tparams, const T * const x, T *residuals) const
{
	Eigen::Map<Eigen::Matrix<T, 2, 1>> distortion((T*)_distortion);
	Eigen::Map<Eigen::Matrix<T, 3, 1>> tparams((T*)_tparams);

	Eigen::Matrix<T, 3, 1>  xw;
	xw[0] = x[0];
	xw[1] = x[1];
	xw[2] = T(0);

	//Rotate and translate
	Eigen::Matrix<T, 3, 1>  xc;
	ceres::AngleAxisRotatePoint(rparams, xw.data(), xc.data());

	xc += tparams;

	//Normalize
	Eigen::Matrix<T, 2, 1> xn = xc.hnormalized();

	//Distort
	Eigen::Matrix<T, 2, 1> xd;
	RadialCameraDistortionModel::DistortPoint(mMaxRadiusSq, distortion, xn, xd);

	//Intrinsics
	Eigen::Matrix<T, 2, 1> pd;
	pd[0] = kparams[0] * xd[0] + kparams[2];
	pd[1] = kparams[1] * xd[1] + kparams[3];

	//Residuals
	residuals[0] = (T(mImgPoint.x()) - pd[0]) / T(mScale);
	residuals[1] = (T(mImgPoint.y()) - pd[1]) / T(mScale);
	return true;
}



void CalibratedBundleAdjuster::addFrameToAdjust(Keyframe &newFrame)
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

bool CalibratedBundleAdjuster::isInlier(const FeatureMeasurement &measurement, const Eigen::Matrix<double, 1, 6> &pose, const Eigen::Vector2d &position)
{
	CalibratedReprojectionError err(mImageSize, measurement);
	Eigen::Vector2d residuals;
	err(mParamsDistortion.data(), mParamsK.data() , pose.data(), pose.data() + 3, position.data(), residuals.data());

	if(residuals.squaredNorm() < mOutlierPixelThresholdSq)
		return true;
	else
		return false;
}

void CalibratedBundleAdjuster::getInliers(int &inlierCount)
{
	inlierCount = 0;
	for (auto &mp : mMeasurementsInProblem)
	{
		auto &m = *mp;
		auto &poseParams = mParamsPoses.find(&m.getKeyframe())->second;
		auto &featureParams = mParamsFeatures.find(&m.getFeature())->second;

		if (isInlier(m, poseParams, featureParams))
			inlierCount++;
	}
}

Eigen::Matrix<double, 1, 6> &CalibratedBundleAdjuster::getPoseParams(Keyframe *framep)
{
	auto &frame = *framep;
	auto itNew = mParamsPoses.emplace(&frame, Eigen::Matrix<double,1,6>());
	auto &params = itNew.first->second;
	if (itNew.second)
	{
		//Is new, create
		Eigen::Matrix3dr Rd = frame.mPose3DR.cast<double>();
		ceres::RotationMatrixToAngleAxis(CeresUtils::FixedRowMajorAdapter3x3<const double>(Rd.data()), &params[0]);
		params[3] = frame.mPose3DT[0];
		params[4] = frame.mPose3DT[1];
		params[5] = frame.mPose3DT[2];
	}

	return params;
}

Eigen::Vector2d &CalibratedBundleAdjuster::getFeatureParams(Feature *featurep)
{
	auto &feature = *featurep;
	auto itNew = mParamsFeatures.emplace(&feature, Eigen::Vector2d());
	auto &params = itNew.first->second;
	if (itNew.second)
	{
		//Is new, create
		params[0] = feature.mPosition3D[0];
		params[1] = feature.mPosition3D[1];
	}

	return params;
}

bool CalibratedBundleAdjuster::bundleAdjust()
{
	ProfileSection s("calibratedBundleAdjust");

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

		assert(mMap->getKeyframes().size() >= 2);
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
				problem.AddParameterBlock(params.data(), 3);
				//problem.AddParameterBlock(params.data() + 3, 3, new Fixed3DNormParametrization(1));
				problem.AddParameterBlock(params.data() + 3, 3);
			}
			else
			{
				problem.AddParameterBlock(params.data(), 3);
				problem.AddParameterBlock(params.data() + 3, 3);
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

		//K params
		mParamsK[0] = mK(0, 0);
		mParamsK[1] = mK(1, 1);
		mParamsK[2] = mK(0, 2);
		mParamsK[3] = mK(1, 2);
		problem.AddParameterBlock(mParamsK.data(), mParamsK.size());
		//problem.SetParameterBlockConstant(mParamsK.data());
		//options.linear_solver_ordering->AddElementToGroup(kparams.data(), 1);

		//Distortion params
		mImageSize = (**mFramesToAdjust.begin()).getImageSize(); //Image size is needed to determine the maximum radius for distortion
		problem.AddParameterBlock(mParamsDistortion.data(), mParamsDistortion.size());

		//Gather measurements
		mMeasurementsInProblem.clear();
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
				problem.AddParameterBlock(poseParams.data(), 3);
				problem.AddParameterBlock(poseParams.data() + 3, 3);
				problem.SetParameterBlockConstant(poseParams.data());
				problem.SetParameterBlockConstant(poseParams.data() + 3);
				//options.linear_solver_ordering->AddElementToGroup(poseParams.data(), 1);
				//options.linear_solver_ordering->AddElementToGroup(poseParams.data()+3, 1);
			}

			ceres::LossFunction *lossFunc_i = NULL;
			lossFunc_i = new ceres::CauchyLoss(mOutlierPixelThreshold);

			problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<CalibratedReprojectionError, CalibratedReprojectionError::kResidualCount, 2, 4, 3, 3, 2>(
				new CalibratedReprojectionError(mImageSize, m)),
				lossFunc_i, mParamsDistortion.data(), mParamsK.data(), poseParams.data(), poseParams.data() + 3, featureParams.data());
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

	MYAPP_LOG << "Calibrated BA report:\n" << summary.FullReport();
	MYAPP_LOG << "Calibrated BA K: " << mParamsK.transpose() << "\n";
	MYAPP_LOG << "Calibrated BA distortion: " << mParamsDistortion.transpose() << "\n";
	
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

	//Again
	//problem.SetParameterBlockVariable(mParamsK.data());
	//ceres::Solve(options, &problem, &summary);

	//MYAPP_LOG << "Calibrated BA report 2:\n" << summary.FullReport();
	//MYAPP_LOG << "Calibrated BA K: " << mParamsK << "\n";


	//Update K
	mK(0, 0) = mParamsK[0];
	mK(1, 1) = mParamsK[1];
	mK(0, 2) = mParamsK[2];
	mK(1, 2) = mParamsK[3];

	//Update pose
	for (auto &p : mParamsPoses)
	{
		auto &frame = *p.first;
		auto &params = p.second;

		Eigen::Matrix3dr Rd;
		ceres::AngleAxisToRotationMatrix(&params[0], CeresUtils::FixedRowMajorAdapter3x3<double>(Rd.data()));
		frame.mPose3DR = Rd.cast<float>();
		frame.mPose3DT[0] = (float)params[3];
		frame.mPose3DT[1] = (float)params[4];
		frame.mPose3DT[2] = (float)params[5];
	}

	//Update positions
	for (auto &p : mParamsFeatures)
	{
		auto &feature = *p.first;
		auto &params = p.second;

		feature.mPosition3D[0] = (float)params[0];
		feature.mPosition3D[1] = (float)params[1];
		feature.mPosition3D[2] = 0;
	}
	return true;
}

} /* namespace dtslam */
