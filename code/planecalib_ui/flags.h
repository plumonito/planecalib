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
	DECLARE_int32(DriverCameraId);
	DECLARE_string(DriverDataPath);
	DECLARE_string(DriverVideoFile);
	DECLARE_string(DriverSequenceFormat);
	DECLARE_int32(DriverSequenceStartIdx);
	DECLARE_int32(DriverDropFrames);
	DECLARE_int32(DriverMaxImageWidth);
	DECLARE_bool(DriverSingleThreaded);
	DECLARE_string(DriverRecordPath);
	//
	//DECLARE_double(MapDrawScale);
}

#endif 
