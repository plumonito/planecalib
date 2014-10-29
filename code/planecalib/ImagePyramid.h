/*
 * ImagePyramid.h
 *
 *  Created on: 8.2.2014
 *      Author: dan
 */

#ifndef IMAGEPYRAMID_H_
#define IMAGEPYRAMID_H_

#include <vector>
#include <memory>
#include <opencv2/core.hpp>

namespace planecalib {

template<class T>
class ImagePyramid {
public:
	static int GetOctaveCount(int level0Width, int maxTopLevelWidth);

	int getOctaveCount() const {return (int)mOctaves.size();}

	cv::Mat_<T> &operator [](int octave) {return mOctaves[octave];}
	const cv::Mat_<T> &operator [](int octave) const {return mOctaves[octave];}

	cv::Mat_<T> &getTopLevel() {return mOctaves.back();}
	const cv::Mat_<T> &getTopLevel() const {return mOctaves.back();}

	void create(const cv::Mat_<T> &level0, int maxTopLevelWidth);

protected:
	std::vector<cv::Mat_<T>> mOctaves;
};

typedef ImagePyramid<uchar> ImagePyramid1b;
typedef ImagePyramid<cv::Vec2b> ImagePyramid2b;
typedef ImagePyramid<cv::Vec3b> ImagePyramid3b;

}

#endif /* IMAGEPYRAMID_H_ */
