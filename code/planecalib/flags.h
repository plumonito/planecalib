/*
 * flags.h
 *
 *  Created on: 29.6.2014
 *      Author: dan
 */

#ifndef dtslam_FLAGS_H_
#define dtslam_FLAGS_H_

#undef GFLAGS_DLL_DECLARE_FLAG
#define GFLAGS_DLL_DECLARE_FLAG
#include <gflags/gflags.h>

namespace planecalib
{
	DECLARE_int32(PyramidMaxTopLevelWidth);
	DECLARE_int32(SBIMaxWidth);
	DECLARE_int32(FeatureDetectorThreshold);
	DECLARE_int32(FrameKeypointGridSize);


	DECLARE_int32(TrackerMaxFeatures);
	DECLARE_int32(TrackerMaxFeaturesPerOctave);
	DECLARE_int32(TrackerMinMatchCount);

	DECLARE_int32(TrackerSelectFeaturesGridSize);
	DECLARE_int32(MatcherPixelSearchDistance);
	DECLARE_int32(TrackerOutlierPixelThreshold);


	DECLARE_int32(MatcherNonMaximaPixelSize);
	DECLARE_int32(MatcherMaxZssdScore);
	DECLARE_double(MatcherBestScorePercentThreshold);
	DECLARE_double(WarperMaxCornerDrift);


	DECLARE_double(ExpanderMinTriangulationAngle);
	DECLARE_int32(ExpanderMinNewTriangulationsForKeyFrame);
	DECLARE_double(ExpanderNewCoverageRatioForKeyFrame);


	DECLARE_int32(PoseMinRansacIterations);
	DECLARE_int32(PoseMaxRansacIterations);
	DECLARE_int32(PoseMinTrackLength);
	DECLARE_double(PoseStableRatioThreshold);

	DECLARE_bool(DisableBA);
	DECLARE_int32(GlobalBAFrameCount);
}

#endif 
