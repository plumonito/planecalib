/*
 * BundleAdjuster.cpp
 *
 *  Created on: 6.3.2014
 *      Author: dan
 */

#include "BundleAdjuster.h"
#include "gflags/gflags.h"

#include <opencv2/calib3d.hpp>
#include <ceres/ceres.h>
#include <mutex>
#include "shared_mutex.h"
#include "Profiler.h"
#include "CeresParametrization.h"
#include "PoseTracker.h"
#include "CameraDistortionModel.h"
#include "CameraModelCeres.h"

namespace planecalib
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// P0Regularizer class

class P0Regularizer
{
public:
	P0Regularizer(const double scale, const Eigen::Vector2d &ref) :
		mScale(scale), mRef(ref)
	{
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const int kResidualCount = 2;

	template<class T>
	bool operator() (const T * const _p0, T *residuals) const;

protected:
	const Eigen::Vector2d mRef;
	const double mScale;
};

template<class T>
bool P0Regularizer::operator() (const T * const _p0, T *residuals) const
{
	T scale(mScale);

	residuals[0] = scale*(T(mRef[0]) - T(_p0[0]));
	residuals[1] = scale*(T(mRef[1]) - T(_p0[1]));
	
	return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// ReprojectionError class

class ReprojectionError
{
public:
	typedef CameraModel::TDistortionModel::TParamVector TDistortionParamVector;

	ReprojectionError(const FeatureMeasurement &m):
		ReprojectionError(m.getOctave(), m.getPosition())
	{
	}

	ReprojectionError(const int octave, const Eigen::Vector2f &imgPoint) :
		mScale(1 << octave), mImgPoint(imgPoint)
	{
	}

	static const int kResidualCount = 2;

	template<class T>
	bool operator() (const T * const _p0, const T * const _distortion, const T * const _homography, const T * const _x, T *residuals) const;

	bool operator() (const Eigen::Vector2d &p0, const TDistortionParamVector &distortion, const Eigen::Matrix3dr &homography, const Eigen::Vector2d &x, Eigen::Vector2d &residuals) const
	{
		return (*this)(p0.data(), distortion.data(), homography.data(), x.data(), residuals.data());
	}

protected:
	const int mScale;
	const Eigen::Vector2f mImgPoint;
};

template<class T>
bool ReprojectionError::operator () (const T * const _p0, const T * const _distortion, const T * const _homography, const T * const _x, T *residuals) const
{
	Eigen::Map<Eigen::Matrix<T, TDistortionParamVector::RowsAtCompileTime, 1>> distortion((T*)_distortion);
	Eigen::Map<Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> homography((T*)_homography);
	Eigen::Map<Eigen::Matrix<T, 2, 1>> x((T*)_x);
	Eigen::Map<Eigen::Matrix<T, 2, 1>> p0((T*)_p0);


	//Homography
	//const Eigen::Matrix<T, 3, 1> p = homography*x.homogeneous();
	Eigen::Matrix<T, 3, 1> xh;
	xh[0] = homography(0, 0) * x[0] + homography(0, 1) * x[1] + homography(0, 2);
	xh[1] = homography(1, 0) * x[0] + homography(1, 1) * x[1] + homography(1, 2);
	xh[2] = homography(2, 0) * x[0] + homography(2, 1) * x[1] + homography(2, 2);

	//Camera model
	const Eigen::Matrix<T, 2, 1> focal(T(1), T(1));
	Eigen::Matrix<T, 2, 1> p;
	CameraModel::ProjectFromWorld(p0, distortion, focal, xh, p);
	
	//Residuals
	residuals[0] = (T(mImgPoint.x()) - p[0]) / T(mScale);
	residuals[1] = (T(mImgPoint.y()) - p[1]) / T(mScale);
	return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// BundleAdjusterclass

void BundleAdjuster::setCamera(CameraModel *camera)
{
	mCamera = camera;

	mParamsPrincipalPoint = mCamera->getPrincipalPoint().cast<double>();
	mParamsDistortion = mCamera->getDistortionModel().getParams();
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

bool BundleAdjuster::isInlier(const FeatureMeasurement &measurement)
{
	Eigen::Vector2d residuals;
	ReprojectionError err(measurement);
	err(mParamsPrincipalPoint, mParamsDistortion, measurement.getKeyframe().mParamsPose, measurement.getFeature().mParams, residuals);
	
	return residuals.squaredNorm() <= mOutlierPixelThresholdSq;
}

void BundleAdjuster::getInliers(int &inlierCount, std::vector<FeatureMeasurement *> &outliers)
{
	inlierCount = 0;
	outliers.clear();
	for(auto &mp : mMeasurementsInProblem)
	{
		auto &m = *mp;
		if (isInlier(m))
			inlierCount++;
		else
			outliers.push_back(mp);
	}
}

bool BundleAdjuster::bundleAdjust()
{
	ProfileSection s("bundleAdjust");

	if (mFramesToAdjust.empty())
		return true;

	//BA ceres problem
	ceres::Solver::Options options;
	if (mOnlyDistortion)
	{
		options.linear_solver_type = ceres::DENSE_QR;
	}
	else
	{
		options.linear_solver_type = ceres::SPARSE_SCHUR;
		options.preconditioner_type = ceres::SCHUR_JACOBI;
	}
	//options.linear_solver_type = ceres::CGNR;
	//options.linear_solver_type = ceres::DENSE_QR;

	//options.dense_linear_algebra_library_type = ceres::LAPACK;
	//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.max_num_iterations = 500;
	options.num_threads = 4;
	options.num_linear_solver_threads = 4;
	options.logging_type = ceres::SILENT;

	options.minimizer_progress_to_stdout = false;

	options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering());

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
			auto &params = frame.mParamsPose;

			//Is new, create
			params = frame.getPose().cast<double>();

			//Add pose as parameter block
			problem.AddParameterBlock(params.data(), 9);
			//problem.AddParameterBlock(params.data(), 9, new Fixed9DNormParametrization(1));
			options.linear_solver_ordering->AddElementToGroup(params.data(), 0);
			if (&frame == mMap->getKeyframes().begin()->get())
			{
				//First key frame in region, scale fixed
				//problem.SetParameterBlockConstant(params.data());
			}
			else if (mOnlyDistortion)
			{
				problem.SetParameterBlockConstant(params.data());
			}			

		}

		//Prepare features
		for (auto &featurep : mFeaturesToAdjust)
		{
			auto &feature = *featurep;

			//Create and init params
			auto &params = feature.mParams;

			//Is new, create
			params = feature.getPosition().cast<double>();

			//Add feature as parameter block
			problem.AddParameterBlock(params.data(), 2);
			if (mOnlyDistortion)
				problem.SetParameterBlockConstant(params.data());
			options.linear_solver_ordering->AddElementToGroup(params.data(), 0);
		}

		//Distortion params
		problem.AddParameterBlock(mParamsPrincipalPoint.data(), 2);
		options.linear_solver_ordering->AddElementToGroup(mParamsPrincipalPoint.data(), 0);
		if (mOnlyDistortion)
			problem.SetParameterBlockConstant(mParamsPrincipalPoint.data());

		problem.AddParameterBlock(mParamsDistortion.data(), TDistortionParamVector::SizeAtCompileTime);
		//problem.SetParameterBlockConstant(mParamsDistortion.data());
		options.linear_solver_ordering->AddElementToGroup(mParamsDistortion.data(), 0);

		//Gather measurements
		mMeasurementsInProblem.clear();
		mMeasurementsInProblem.reserve(4 * mFeaturesToAdjust.size());
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
			auto &featureParams = feature.mParams;

			Keyframe &frame = m.getKeyframe();
			auto &poseParams = frame.mParamsPose;

			//Is this frame outside of bundle adjustment?
			//if (mFramesToAdjust.find(&frame) == mFramesToAdjust.end())
			//{
			//	problem.AddParameterBlock(poseParams.data(), 9);
			//	problem.SetParameterBlockConstant(poseParams.data());
			//	//options.linear_solver_ordering->AddElementToGroup(poseParams.data(), 1);
			//}

			ceres::LossFunction *lossFunc_i = NULL;
			lossFunc_i = new ceres::CauchyLoss(mOutlierPixelThreshold);

			problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<ReprojectionError, ReprojectionError::kResidualCount, 2, TDistortionParamVector::SizeAtCompileTime, 9, 2>(
				new ReprojectionError(m)),
				lossFunc_i, mParamsPrincipalPoint.data(), mParamsDistortion.data(), poseParams.data(), featureParams.data());
		}

		//Add p0 regularizer
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<P0Regularizer, P0Regularizer::kResidualCount, 2>(
			new P0Regularizer(1, mParamsPrincipalPoint)),
			NULL, mParamsPrincipalPoint.data());
	}

	//Get inliers before
	int inlierCount;
	std::vector<FeatureMeasurement *> outliers;
	getInliers(inlierCount, outliers);
	MYAPP_LOG << "BA inlier count before: " << inlierCount << " / " << mMeasurementsInProblem.size() << "\n";

	//No locks while ceres runs
	//Non-linear minimization!
	ceres::Solver::Summary summary;
	{
		ProfileSection ssolve("solve");
		ceres::Solve(options, &problem, &summary);
	}

	MYAPP_LOG << "BA report:\n" << summary.FullReport();
	MYAPP_LOG << "P0: " << mParamsPrincipalPoint.transpose() << "\n";
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

	getInliers(inlierCount, outliers);
	MYAPP_LOG << "BA inlier count after: " << inlierCount << " / " << mMeasurementsInProblem.size() << "\n";

	//Update camera
	mCamera->getPrincipalPoint() = mParamsPrincipalPoint.cast<float>();
	mCamera->getDistortionModel().setParams(mParamsDistortion);

	//Update pose
	for (auto &framep : mFramesToAdjust)
	{
		auto &frame = *framep;
		auto &params = frame.mParamsPose;

		frame.setPose(params.cast<float>());
	}

	//Update positions
	for (auto &featurep : mFeaturesToAdjust)
	{
		auto &feature = *featurep;
		auto &params = feature.mParams;

		feature.setPosition(params.cast<float>());
	}

	//Remove outliers
	for (auto mp : outliers)
	{
		auto &frameVec = mp->getKeyframe().getMeasurements();
		for (auto it = frameVec.begin(), end = frameVec.end(); it != end; ++it)
		{
			if (*it == mp)
			{
				frameVec.erase(it);
				break;
			}
		}

		auto &featuresVec = mp->getFeature().getMeasurements();
		for (auto it = featuresVec.begin(), end = featuresVec.end(); it != end; ++it)
		{
			if (it->get() == mp)
			{
				featuresVec.erase(it);
				break;
			}
		}
	}
	MYAPP_LOG << "Removed " << outliers.size() << " outliers\n";

	return true;
}

} /* namespace dtslam */
