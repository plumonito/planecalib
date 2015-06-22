
#ifndef HOMOGRAPHYCALIBRATION_H_
#define HOMOGRAPHYCALIBRATION_H_

#include "eutils.h"
#include "CameraModel.h"
#include "BaseRansac.h"

namespace planecalib {

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HomographyCalibrationRansac
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct HomographyCalibrationIterationData
{
	std::vector<MatchReprojectionErrors> reprojectionErrors;
};

class HomographyCalibrationRansac : public BaseRansac<double, HomographyCalibrationIterationData, 1>
{
public:
	HomographyCalibrationRansac() {}
	~HomographyCalibrationRansac() {}

	void setData(const std::vector<Eigen::Matrix3d> *homographies);

	std::vector<double> modelFromMinimalSet(const std::vector<int> &constraintIndices);
	void getInliers(const double &model, int &inlierCount, float &errorSumSq, HomographyCalibrationIterationData &data);

protected:
	const std::vector<Eigen::Matrix3d> *mHomographies;
};


class HomographyCalibration
{
public:
	HomographyCalibration() : mUseNormalizedConstraints(true), mFixPrincipalPoint(true), mVerbose(false), mInitialAlpha(0), mNormal(0,0,0)
	{
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool getUseNormalizedConstraints()  const { return mUseNormalizedConstraints; }
	void setUseNormalizedConstraints(bool value)  { mUseNormalizedConstraints = value; }

	bool getFixPrincipalPoint()  const { return mFixPrincipalPoint; }
	void setFixPrincipalPoint(bool value)  { mFixPrincipalPoint = value; }

	bool getVerbose()  const { return mVerbose; }
	void setVerbose(bool value)  { mVerbose = value; }

	double getInitialAlpha() const { return mInitialAlpha; }
	const Eigen::Vector2d &getPrincipalPoint() const { return mPrincipalPoint; }
	//const CameraModel::TDistortionModel::TParamVector &getDistortionParams() const { return mDistortionParams; }
	const Eigen::Vector2d &getFocalLengths() const { return mFocalLengths; }

	const Eigen::Vector3d &getNormal() const { return mNormal; }

	void initFromCamera(const CameraModel &camera);
	void updateCamera(CameraModel &camera) const;

	void calibrate(const std::vector<Eigen::Matrix3fr> &H);

protected:
	bool mUseNormalizedConstraints;
	bool mFixPrincipalPoint;
	bool mVerbose;

	double mInitialAlpha;
	Eigen::Vector2d mPrincipalPoint;
	//CameraModel::TDistortionModel::TParamVector mDistortionParams;
	Eigen::Vector2d mFocalLengths;
	Eigen::Vector3d mNormal;

	void calibrateLinear(const std::vector<Eigen::Matrix3d> &H);
	void calibrateNonLinear(const std::vector<Eigen::Matrix3d> &H);
};

}

#endif 
