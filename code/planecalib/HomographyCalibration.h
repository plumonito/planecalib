
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

	Eigen::Matrix3fr calibrate(const std::vector<Eigen::Matrix3fr> &H, const Eigen::Vector2i &imageSize);
};

}

#endif 
