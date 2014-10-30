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
namespace Eigen
{
	typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3fr;
	typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> Matrix4fr;
}

#include <cassert>
#include <opencv2/core/core.hpp>

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
		
		return Eigen::Vector2f(p.x() / p.z(), p.y() / p.z());
	}

	static Eigen::Vector2f HomographyPoint(const Eigen::Matrix3f &H, const Eigen::Vector2f &p)
	{
		
		return NormalizePoint(H*Eigen::Vector3f(p.x(), p.y(), 1.0f));
	}

	static Eigen::Vector2i FromSize(const cv::Size2i &sz) 
	{
		return Eigen::Vector2i(sz.width, sz.height);
	}
	static cv::Size2i ToSize(const Eigen::Vector2i &sz)
	{
		return cv::Size2i(sz.x(), sz.y());
	}
};

}

#endif /* CVUTILS_H_ */
