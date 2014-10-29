/*
 * SequenceDataSource.h
 *
 *  Created on: 22.3.2014
 *      Author: dan
 */

#ifndef SEQUENCEDATASOURCE_H_
#define SEQUENCEDATASOURCE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <GL/glew.h>
#include "ImageDataSource.h"

namespace planecalib {

class SequenceDataSource: public ImageDataSource
{
public:
	SequenceDataSource(void);
    ~SequenceDataSource(void);

    bool open(const std::string &sequenceFormat, int startIdx);
    void close(void);

    void dropFrames(int count);
    bool update(void);

private:
    std::string mSequenceFormat;
    int mCurrentFrameIdx;

    cv::Mat readImage(int idx);
};

} /* namespace planecalib */

#endif /* SEQUENCEDATASOURCE_H_ */
