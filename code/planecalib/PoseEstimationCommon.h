#ifndef POSEESTIMATIONCOMMON_H_
#define POSEESTIMATIONCOMMON_H_

#include <vector>
#include "flags.h"

namespace planecalib {

struct MatchReprojectionErrors
{
	MatchReprojectionErrors():
		isInlier(false)
	{}

	bool isInlier;
	float reprojectionErrorsSq;
};

}

#endif
