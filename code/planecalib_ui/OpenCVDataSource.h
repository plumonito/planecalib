/*
 * VideoImageDataSource.h
 *
 *  Created on: Jul 10, 2013
 *      Author: danielh
 */

#ifndef OPENCV_IMAGEDATASOURCE_H_
#define OPENCV_IMAGEDATASOURCE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <GL/glew.h>
#include "ImageDataSource.h"

namespace planecalib
{

class OpenCVDataSource: public ImageDataSource
{
public:
    OpenCVDataSource(void);
    ~OpenCVDataSource(void);

    bool open(const std::string &videoFile);
    bool open(int deviceId);
    void close(void);

    void dropFrames(int count);
    bool update(void);

private:
    cv::VideoCapture mOpenCVCamera;

    bool finishOpen();
};

}

#endif /* VIDEOIMAGEDATASOURCE_H_ */
