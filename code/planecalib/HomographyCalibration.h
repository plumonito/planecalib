
#ifndef HOMOGRAPHYCALIBRATION_H_
#define HOMOGRAPHYCALIBRATION_H_

#include "eutils.h"

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
	
	const Eigen::Matrix3fr &getK() const { return mK; }
	const Eigen::Vector3d &getNormal() const { return mNormal; }

	void calibrate(const Eigen::Vector2f &p0, const std::vector<Eigen::Matrix3fr> &H);

protected:
	bool mUseNormalizedConstraints;
	bool mFixPrincipalPoint;

	double mInitialAlpha;
	Eigen::Matrix3fr mK;
	Eigen::Vector3d mNormal;
};

}

#endif 
