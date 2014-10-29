/*
 * cvutils.cpp
 *
 *  Created on: 9.2.2014
 *      Author: dan
 */

#include "cvutils.h"
#include <opencv2/imgproc.hpp>

namespace planecalib
{

void cvutils::CalculateDerivatives(const cv::Mat1b &img, cv::Mat1s &dx, cv::Mat1s &dy)
{
	static const int kShiftFactor = 1;
	cv::Sobel(img, dx, CV_16S, 1, 0, 1);
	cv::Sobel(img, dy, CV_16S, 0, 1, 1);

//	static const int kShiftFactor = 3;
//	cv::Sobel(img, dx, CV_16S, 1, 0, 3);
//	cv::Sobel(img, dy, CV_16S, 0, 1, 3);

//	static const int kShiftFactor = 5;
//	cv::Scharr(img, dx, CV_16S, 1, 0);
//	cv::Scharr(img, dy, CV_16S, 0, 1);

	for(int y=0; y<img.rows; y++)
	{
		short *dxRow=dx[y];
		short *dyRow=dy[y];
		for(int x=0; x<img.cols; x++)
		{
			dxRow[x] >>= kShiftFactor;
			dyRow[x] >>= kShiftFactor;
		}
	}
}

void cvutils::DownsampleImage(const cv::Mat &img, cv::Mat &res, int count)
{
	res = img;
	cv::Mat temp;
	for(int i=0; i<count;i++)
	{
		cv::pyrDown(res,temp);
		res=temp;
	}
}

}
