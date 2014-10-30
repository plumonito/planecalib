/*
 * cvutils.h
 *
 *  Created on: 6.2.2014
 *      Author: dherrera
 */

#ifndef EUTILS_H_
#define EUTILS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f);

#include <cassert>

namespace planecalib
{

class eutils
{
public:
	static bool IsInside(const Eigen::Vector2i &imageSize, const Eigen::Vector2f &p)
	{
		if (p.x() < 0 || p.y() < 0 || p.x() >= imageSize.x() || p.y() >= imageSize.y())
			return false;
		else
			return true;
	}

	static Eigen::Vector2f NormalizePoint(const Eigen::Vector3f &p)
	{
		
		return Eigen::Vector2f(p.x() / p.z(), p.x() / p.z());
	}

	static Eigen::Vector2f HomographyPoint(const Eigen::Matrix3f &H, const Eigen::Vector2f &p)
	{
		
		return NormalizePoint(H*Eigen::Vector3f(p.x(), p.y(), 1.0f));
	}
};

}

#endif /* CVUTILS_H_ */
