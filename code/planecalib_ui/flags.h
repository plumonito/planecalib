/*
 * flags.h
 *
 *  Created on: 29.6.2014
 *      Author: dan
 */

#ifndef planecalib_FLAGS_UI_H_
#define planecalib_FLAGS_UI_H_

//#undef GFLAGS_DLL_DEFINE_FLAG
//#define GFLAGS_DLL_DEFINE_FLAG
#include <gflags/gflags.h>

namespace planecalib
{
	DECLARE_int32(CameraId);
	DECLARE_string(VideoFile);
	DECLARE_string(ImageSequenceFormat);
	DECLARE_int32(ImageSequenceStartIdx);
	DECLARE_int32(DropFrames);
	DECLARE_int32(InputMaxImageWidth);
	DECLARE_bool(SingleThreaded);
	DECLARE_string(RecordPath);
	DECLARE_string(RecordVideoFile);
	//
	//DECLARE_double(MapDrawScale);
}

#endif 
