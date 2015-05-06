#ifndef BOUGUETINTERFACE_H_
#define BOUGUETINTERFACE_H_

#include <Eigen/Dense>
#include <random>
#include <memory>
#include "planecalib/eutils.h"
#include "planecalib/CameraModel.h"

namespace planecalib
{

class Map;

class BouguetInterface
{
public:
	BouguetInterface()
	{}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	std::unique_ptr<Map> loadCalib(const std::string &filename);
	std::unique_ptr<Map> loadValidation(const CameraModel &camera, const std::string &filename);
};

}

#endif 
