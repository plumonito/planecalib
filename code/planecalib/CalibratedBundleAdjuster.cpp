
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
//#include "EpipolarSegmentError.h"
#include "PoseTracker.h"

namespace planecalib
{


class CalibratedReprojectionError
{
public:
	CalibratedReprojectionError(const FeatureMeasurement &m) :
		CalibratedReprojectionError(m.getOctave(), m.getPosition())
	{
	}

	CalibratedReprojectionError(const int octave, const Eigen::Vector2f &imgPoint) :
		mScale(1<<octave), mImgPoint(imgPoint) 
	{
	}

	static const int kResidualCount = 2;

	template<class T>
	bool operator() (const T * const kparams, const T * const rparams, const T * const tparams, const T * const x, T *residuals) const;

protected:
	const int mScale;
	const Eigen::Vector2f mImgPoint;
};

template<class T>
bool CalibratedReprojectionError::operator () (const T * const kparams, const T * const rparams, const T * const tparams, const T * const x, T *residuals) const
{
	T xw[3];
	xw[0] = x[0];
	xw[1] = x[1];
	xw[2] = T(0);

	//Rotate and translate
	T xc[3];
	ceres::AngleAxisRotatePoint(rparams, xw, xc);

	xc[0] += tparams[0];
	xc[1] += tparams[1];
	xc[2] += tparams[2];

	//Normalize + K
	T p[2];
	p[0] = kparams[0] * xc[0] / xc[2] + kparams[1];
	p[1] = kparams[0] * xc[1] / xc[2] + kparams[2];

	//Residuals
	residuals[0] = (T(mImgPoint.x()) - p[0]) / T(mScale);
	residuals[1] = (T(mImgPoint.y()) - p[1]) / T(mScale);
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
			if(feature.getMeasurements().size() > 1)
			{
				mFeaturesToAdjust.insert(&feature);
			}
		}
	}
}

bool CalibratedBundleAdjuster::bundleAdjust()
{
	ProfileSection s("bundleAdjust");

	if(mFramesToAdjust.empty())
		return true;

	//Gather relevant measurements
	Eigen::Vector3d kparams;
	std::vector<Eigen::Matrix<double, 1, 6>> paramsPoses;
	std::vector<Eigen::Vector2d> paramsFeatures;
	std::unordered_map<Keyframe *, int> frameMap;
	std::unordered_map<Feature *, int> featureMap;

	//Store pose
	paramsPoses.resize(mMap->getKeyframes().size());
	for (int i = 0; i < (int)paramsPoses.size(); i++)
	{
		auto &frame = *mMap->getKeyframes()[i];

		frameMap.insert(std::make_pair(&frame, i));

		Eigen::Matrix<double, 1, 6> &params = paramsPoses[i];


		Eigen::Matrix3dr Rd = frame.mPose3DR.cast<double>();
		ceres::RotationMatrixToAngleAxis(CeresUtils::FixedRowMajorAdapter3x3<const double>(Rd.data()), &params[0]);
		params[3] = frame.mPose3DT[0];
		params[4] = frame.mPose3DT[1];
		params[5] = frame.mPose3DT[2];
	}

	//Store positions
	paramsFeatures.resize(mMap->getFeatures().size());
	for (int i = 0; i < (int)paramsFeatures.size(); i++)
	{
		auto &feature = *mMap->getFeatures()[i];
		
		featureMap.insert(std::make_pair(&feature, i));

		auto &params = paramsFeatures[i];
		params[0] = feature.mPosition3D[0];
		params[1] = feature.mPosition3D[1];
	}

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
			auto it = frameMap.find(&frame);
			auto &params = paramsPoses[it->second];

			//Add pose as parameter block
			if (&frame == mMap->getKeyframes().begin()->get())
			{
				//First key frame in region, scale fixed
				problem.AddParameterBlock(params.data(), 3);
				problem.AddParameterBlock(params.data() + 3, 3, new Fixed3DNormParametrization(1));
			}
			else
			{
				problem.AddParameterBlock(params.data(), 3);
				problem.AddParameterBlock(params.data() + 3, 3);
			}
			options.linear_solver_ordering->AddElementToGroup(params.data(), 1);
			options.linear_solver_ordering->AddElementToGroup(params.data()+3, 1);
		}

		//Prepare features
		for (auto &featurePtr : mFeaturesToAdjust)
		{
			Feature &feature = *featurePtr;

			if (feature.getMeasurements().size() <= 1)
				continue; //Skip if only one measurement

			//Add feature to params list
			auto it = featureMap.find(&feature);
			auto &params = paramsFeatures[it->second];

			//Add 3D feature as parameter block
			problem.AddParameterBlock(params.data(), 2);
			problem.SetParameterBlockConstant(params.data());
			options.linear_solver_ordering->AddElementToGroup(params.data(), 0);
		}

		//K params
		kparams[0] = mK(0, 0);
		kparams[1] = mK(0, 2);
		kparams[2] = mK(1, 2);
		problem.AddParameterBlock(kparams.data(), 3);
		problem.SetParameterBlockConstant(kparams.data());
		options.linear_solver_ordering->AddElementToGroup(kparams.data(), 1);

		//Add all 3D feautres to ceres problem
		for (int i = 0; i < (int)paramsFeatures.size(); i++)
		{
			Feature &feature = *mMap->getFeatures()[i];
			auto &featureParams = paramsFeatures[i];

			if (feature.getMeasurements().size() <= 1)
				continue; //Skip if only one measurement

			//Add all measurements as residual blocks
			for (auto &mPtr : feature.getMeasurements())
			{
				FeatureMeasurement &m = *mPtr;
				Keyframe &frame = m.getKeyframe();

				//Get pose
				auto itPose = frameMap.find(&frame);
				auto &poseParams = paramsPoses[itPose->second];

				//Is this frame outside of bundle adjustment?
				if (mFramesToAdjust.find(&frame) == mFramesToAdjust.end())
				{
					problem.AddParameterBlock(poseParams.data(), 3);
					problem.AddParameterBlock(poseParams.data()+3, 3);
					problem.SetParameterBlockConstant(poseParams.data());
					problem.SetParameterBlockConstant(poseParams.data()+3);
					options.linear_solver_ordering->AddElementToGroup(poseParams.data(), 1);
					options.linear_solver_ordering->AddElementToGroup(poseParams.data()+3, 1);
				}

				//const int scale = 1<<m.getOctave();
				//const double costScale = (feature.getMeasurements().size() > 3) ? 1e6 : 1.0;
				ceres::LossFunction *lossFunc_i = new ceres::CauchyLoss(mOutlierPixelThreshold);
				//ceres::LossFunction *scaledLoss = new ceres::ScaledLoss(lossFunc_i, costScale, ceres::TAKE_OWNERSHIP);

				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<CalibratedReprojectionError, CalibratedReprojectionError::kResidualCount, 3,3,3,2>(
					new CalibratedReprojectionError(m)),
					lossFunc_i, kparams.data(), poseParams.data(), poseParams.data()+3, featureParams.data());
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

	MYAPP_LOG << "Calibrated BA report:\n" << summary.FullReport();
	MYAPP_LOG << "Calibrated BA K: " << kparams << "\n";
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

	//Update K
	mK(0, 0) = kparams[0];
	mK(0, 2) = kparams[1];
	mK(1, 2) = kparams[2];

	//Update pose
	for (int i = 0; i < (int)paramsPoses.size(); i++)
	{
		auto &frame = *mMap->getKeyframes()[i];

		Eigen::Matrix<double, 1, 6> &params = paramsPoses[i];


		Eigen::Matrix3dr Rd;
		ceres::AngleAxisToRotationMatrix(&params[0], CeresUtils::FixedRowMajorAdapter3x3<double>(Rd.data()));
		frame.mPose3DR = Rd.cast<float>();
		frame.mPose3DT[3] = (float)params[0];
		frame.mPose3DT[4] = (float)params[1];
		frame.mPose3DT[5] = (float)params[2];
	}

	//Update positions
	for (int i = 0; i < (int)paramsFeatures.size(); i++)
	{
		auto &feature = *mMap->getFeatures()[i];

		auto &params = paramsFeatures[i];
		feature.mPosition3D[0] = (float)params[0];
		feature.mPosition3D[1] = (float)params[1];
		feature.mPosition3D[2] = 0;
	}
	return true;
}

} /* namespace dtslam */
