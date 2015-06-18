
#ifndef HOMOGRAPHYCALIBRATION_H_
#define HOMOGRAPHYCALIBRATION_H_

#include "eutils.h"
#include "CameraModel.h"

namespace planecalib {

class HomographyCalibration
{
public:
	HomographyCalibration() : mUseNormalizedConstraints(true), mInitialAlpha(0), mNormal(0,0,0)
	{
	}

	bool getUseNormalizedConstraints()  const { return mUseNormalizedConstraints; }
	void setUseNormalizedConstraints(bool value)  { mUseNormalizedConstraints = value; }

	bool getFixPrincipalPoint()  const { return mFixPrincipalPoint; }
	void setFixPrincipalPoint(bool value)  { mFixPrincipalPoint = value; }

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

	double mInitialAlpha;
	Eigen::Vector2d mPrincipalPoint;
	//CameraModel::TDistortionModel::TParamVector mDistortionParams;
	Eigen::Vector2d mFocalLengths;
	Eigen::Vector3d mNormal;
};

}

#endif 
