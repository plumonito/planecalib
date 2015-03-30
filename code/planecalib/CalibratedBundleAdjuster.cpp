
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

	//void evalToErrors(const Eigen::Matrix3dr &homography, const Eigen::Vector2d &x, const float errorThreshold, MatchReprojectionErrors &errors) const;

protected:
	const int mScale;
	const Eigen::Vector2f mImgPoint;
};

template<class T>
bool CalibratedReprojectionError::operator () (const T * const kparams, const T * const rparams, const T * const tparams, const T * const x, T *residuals) const
{
	T xc[3];

	//Translate
	ceres::AngleAxisRotatePoint(rparams, x, xc);

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

//void ReprojectionError::evalToErrors(const Eigen::Matrix3dr &homography, const Eigen::Vector2d &x, const float errorThresholdSq, MatchReprojectionErrors &errors) const
//{
//	assert(kResidualCount == 2);
//	
//	double residuals[kResidualCount];
//	this->operator()(homography.data(), x.data(), residuals);
//	
//	errors.reprojectionErrorsSq = (float)(residuals[0] * residuals[0] + residuals[1] * residuals[1]);
//	errors.isInlier = errors.reprojectionErrorsSq < errorThresholdSq;
//}




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
	std::vector<Eigen::Vector3d> paramsFeatures;
	std::unordered_map<Keyframe *, int> frameMap;
	std::unordered_map<Feature *, int> featureMap;

	//Find pose of all cameras
	paramsPoses.resize(mMap->getKeyframes().size());
	for (int i = 0; i < (int)paramsPoses.size(); i++)
	{
		auto &frame = *mMap->getKeyframes()[i];

		frameMap.insert(std::make_pair(&frame, i));

		Eigen::Matrix<double, 1, 6> &pose = paramsPoses[i];

		if (&frame == mMap->getKeyframes().front().get())
		{
			pose.setZero();
		}
		else
		{
			std::vector<cv::Point2f> refPoints, imgPoints;
			for (auto &mPtr : frame.getMeasurements())
			{
				auto &m = *mPtr;
				refPoints.push_back(eutils::ToCVPoint(m.getFeature().getPosition()));
				imgPoints.push_back(eutils::ToCVPoint(m.getPosition()));
			}
			cv::Mat E, R, t, mask;
			E = cv::findEssentialMat(refPoints, imgPoints, mK(0,0), cv::Point2d(mK(0,2),mK(1,2)), cv::RANSAC, 0.999, 1.0, mask);
			cv::recoverPose(E, refPoints, imgPoints, R, t, mK(0, 0), cv::Point2d(mK(0, 2), mK(1, 2)), mask);

			cv::Matx33f Rx = R;
			cv::Vec3f tx = t;

			cv::Matx33d Rd = Rx;
			cv::Vec3d td = tx;
			ceres::RotationMatrixToAngleAxis(ceres::RowMajorAdapter3x3<const double>(Rd.val), &pose[0]);
			pose[3] = td[0];
			pose[4] = td[1];
			pose[5] = td[2];
		}
	}

	//Triangulate all features
	Keyframe &triangulationFrame = *mMap->getKeyframes().back();
	cv::Matx33f cvK = eutils::ToCV(mK.cast<float>().eval());
	cv::Matx34f P1 = cvK*cv::Matx34f::eye();
	cv::Matx34f P2;
	{
		auto &pose = paramsPoses.back();
		cv::Matx33d R;
		ceres::AngleAxisToRotationMatrix(&pose[0], ceres::RowMajorAdapter3x3<double>(R.val));
		cv::Matx34f Rt;
		Rt(0, 0) = (float)R(0, 0);
		Rt(0, 1) = (float)R(0, 1);
		Rt(0, 2) = (float)R(0, 2);
		Rt(1, 0) = (float)R(1, 0);
		Rt(1, 1) = (float)R(1, 1);
		Rt(1, 2) = (float)R(1, 2);
		Rt(2, 0) = (float)R(2, 0);
		Rt(2, 1) = (float)R(2, 1);
		Rt(2, 2) = (float)R(2, 2);
		Rt(0, 3) = (float)pose[3];
		Rt(1, 3) = (float)pose[4];
		Rt(2, 3) = (float)pose[5];
		P2 = cvK*Rt;
	}

	std::vector<cv::Point2f> refPoints;
	std::vector<cv::Point2f> imgPoints;
	refPoints.resize(mMap->getFeatures().size());
	imgPoints.resize(mMap->getFeatures().size());
	for (int i = 0; i < (int)refPoints.size(); i++)
	{
		auto &feature = *mMap->getFeatures()[i];
		refPoints[i] = eutils::ToCVPoint(feature.getPosition());
		imgPoints[i] = eutils::ToCVPoint(eutils::HomographyPoint(triangulationFrame.getPose(), feature.getPosition()));
	}
	cv::Mat1f p4mat;
	cv::triangulatePoints(P1, P2, refPoints, imgPoints, p4mat);

	paramsFeatures.resize(refPoints.size());
	for (int i = 0; i < (int)refPoints.size(); i++)
	{
		auto &feature = *mMap->getFeatures()[i];
		
		featureMap.insert(std::make_pair(&feature, i));

		//assert(fabs(p4mat(3, i)) > 0.0001f);
		Eigen::Vector3d x;
		x[0] = p4mat(0, i) / p4mat(3, i);
		x[1] = p4mat(1, i) / p4mat(3, i);
		x[2] = p4mat(2, i) / p4mat(3, i);
		paramsFeatures[i] = x;
	}

	//BA ceres problem
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	//options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.dense_linear_algebra_library_type = ceres::LAPACK;

	options.max_num_iterations = 500;
	options.num_threads = 4;
	options.num_linear_solver_threads = 4;
	options.logging_type = ceres::SILENT;
	
	options.minimizer_progress_to_stdout = false;

    // in some environments ceres uses std::tr1::shared_ptr, in others
    // it uses std::shared_ptr. Let's keep it simple by not using
    // make_shared.
	//options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering());

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
			problem.AddParameterBlock(params.data(), 3);
			problem.AddParameterBlock(params.data()+3, 3);
			//options.linear_solver_ordering->AddElementToGroup(params.data(), 1);
			//options.linear_solver_ordering->AddElementToGroup(params.data()+3, 1);

			if (&frame == mMap->getKeyframes().begin()->get())
			{
				//First key frame in region, pose fixed
				problem.SetParameterBlockConstant(params.data());
				problem.SetParameterBlockConstant(params.data()+3);
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
				auto it = featureMap.find(&feature);
				auto &params = paramsFeatures[it->second];

				//Add 3D feature as parameter block
				problem.AddParameterBlock(params.data(), 3);
				//options.linear_solver_ordering->AddElementToGroup(params.data(), 0);
			}
		}

		//K params
		kparams[0] = mK(0, 0);
		kparams[1] = mK(0, 2);
		kparams[2] = mK(1, 2);
		problem.AddParameterBlock(kparams.data(), 3);
		//options.linear_solver_ordering->AddElementToGroup(kparams.data(), 1);

		//Add all 3D feautres to ceres problem
		for (int i = 0; i < (int)paramsFeatures.size(); i++)
		{
			Feature &feature = *mMap->getFeatures()[i];
			auto &featureParams = paramsFeatures[i];

			//Add all measurements as residual blocks
			for (auto &mPtr : feature.getMeasurements())
			{
				FeatureMeasurement &m = *mPtr;
				Keyframe &frame = m.getKeyframe();

				//Get pose
				auto itPose = frameMap.find(&frame);
				auto &poseParams = paramsPoses[itPose->second];

				//Is this frame outside of bundle adjustment?
				//if (itNewPose.second)
				//{
				//	problem.AddParameterBlock(poseParams.data(), 9);
				//	options.linear_solver_ordering->AddElementToGroup(poseParams.data(), 1);
				//	problem.SetParameterBlockConstant(poseParams.data());
				//}

				//const int scale = 1<<m.getOctave();
				//const double costScale = (feature.getMeasurements().size() > 3) ? 1e6 : 1.0;
				ceres::LossFunction *lossFunc_i = new ceres::CauchyLoss(mOutlierPixelThreshold);
				//ceres::LossFunction *scaledLoss = new ceres::ScaledLoss(lossFunc_i, costScale, ceres::TAKE_OWNERSHIP);

				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<CalibratedReprojectionError, CalibratedReprojectionError::kResidualCount, 3,3,3,3>(
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
		MYAPP_LOG << "\n\nBA solver failed!!!\n\n" << summary.FullReport();
		return false;
	}
}

} /* namespace dtslam */
