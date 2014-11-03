/*
 * cvutils.h
 *
 *  Created on: 6.2.2014
 *      Author: dherrera
 */

#ifndef CVUTILS_H_
#define CVUTILS_H_

#include <opencv2/core.hpp>
#include <cassert>

namespace planecalib
{

class cvutils
{
public:
	static cv::Matx23f AffineAffine(const cv::Matx23f &aff1, const cv::Matx23f &aff2)
	{
		const float a00 = aff1(0, 0)*aff2(0, 0) + aff1(0, 1)*aff2(1, 0);
		const float a01 = aff1(0, 0)*aff2(0, 1) + aff1(0, 1)*aff2(1, 1);
		const float a10 = aff1(1, 0)*aff2(0, 0) + aff1(1, 1)*aff2(1, 0);
		const float a11 = aff1(1, 0)*aff2(0, 1) + aff1(1, 1)*aff2(1, 1);
		const float a02 = aff1(0, 0)*aff2(0, 2) + aff1(0, 1)*aff2(1, 2) + aff1(0, 2);
		const float a12 = aff1(1, 0)*aff2(0, 2) + aff1(1, 1)*aff2(1, 2) + aff1(1, 2);

		return cv::Matx23f(a00, a01, a02, a10, a11, a12);
	}

	static cv::Point2f HomographyPoint(const cv::Matx33f &H, const cv::Point2f &p)
	{
		const cv::Vec3f v(p.x, p.y, 1);
		const cv::Vec3f Hv = H*v;

		return cv::Point2f(Hv[0] / Hv[2], Hv[1] / Hv[2]);
	}

	static void CalculateDerivatives(const cv::Mat1b &img, cv::Mat1s &dx, cv::Mat1s &dy);

	//static cv::Matx33f SkewSymmetric(const cv::Vec3f &t)
	//{
	//	return cv::Matx33f(0,-t[2],t[1],  t[2],0,-t[0],  -t[1],t[0],0);
	//}

	//static cv::Matx33f RotationX(float angle)
	//{
	//	return cv::Matx33f(cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));
	//}

	//static cv::Matx33f RotationY(float angle)
	//{
	//	return cv::Matx33f(1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle));
	//}

    static void DownsampleImage(const cv::Mat &img, cv::Mat &res, int count);
};

}

#endif /* CVUTILS_H_ */
