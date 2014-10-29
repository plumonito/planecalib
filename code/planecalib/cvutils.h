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
