
#ifndef HOMOGRAPHYCALIBRATION_H_
#define HOMOGRAPHYCALIBRATION_H_

#include "eutils.h"

namespace planecalib {

class HomographyCalibration
{
public:
	HomographyCalibration()
	{
	}

	double getInitialAlpha() const { return mInitialAlpha; }
	const Eigen::Matrix3fr &getK() const { return mK; }
	const Eigen::Vector3d &getNormal() const { return mNormal; }

	void calibrate(const std::vector<Eigen::Matrix3fr> &H, const Eigen::Vector2i &imageSize);

protected:
	double mInitialAlpha;
	Eigen::Matrix3fr mK;
	Eigen::Vector3d mNormal;
};

}

#endif 
