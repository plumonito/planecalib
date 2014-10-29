/*
 * ImagePyramid.cpp
 *
 *  Created on: 8.2.2014
 *      Author: dan
 */

#include "ImagePyramid.h"

#include <opencv2/imgproc.hpp>

namespace planecalib {

template<class T>
int ImagePyramid<T>::GetOctaveCount(int level0Width, int maxTopLevelWidth)
{
	int octave=0;
	while(level0Width > maxTopLevelWidth)
	{
		level0Width = (level0Width+1)/2;
		octave++;
	}
	return octave;
}

template<class T>
void ImagePyramid<T>::create(const cv::Mat_<T> &level0, int maxTopLevelWidth)
{
	mOctaves.clear();

	int octave=0;
	mOctaves.push_back(level0);
	while(mOctaves[octave].cols > maxTopLevelWidth)
	{
		mOctaves.push_back(cv::Mat_<T>());
		cv::pyrDown(mOctaves[octave], mOctaves[octave+1]);
		octave++;
	}
}

template class ImagePyramid<uchar>;
template class ImagePyramid<cv::Vec2b>;
template class ImagePyramid<cv::Vec3b>;


} 
