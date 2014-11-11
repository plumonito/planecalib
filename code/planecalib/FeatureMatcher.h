/*
 * featurematcher.h
 *
 *  created on: 9.2.2014
 *      author: dan
 */

#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include <memory>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include "eutils.h"
#include "Map.h"

namespace planecalib {

///////////////////////////////////
// classes

	class FeatureMatch
{
public:
	FeatureMatch()
	{
	}

	FeatureMatch(const FeatureMeasurement *source, int octave_, const Eigen::Vector2f &position_, const uchar *descriptor_, int tracklength_) :
		mSourceMeasurement(source), mOctave(octave_), mPosition(position_), mDescriptor(descriptor_), mTrackLength(tracklength_)
	{}

	const FeatureMeasurement *getSourceMeasurement() const { return mSourceMeasurement; }
	const Feature &getFeature() const { return mSourceMeasurement->getFeature(); }
	const Eigen::Vector2f &getPosition() const { return mPosition; }
	const uchar *getDescriptor() const { return mDescriptor; }
	int getOctave() const { return mOctave; }
	int getTrackLength() const { return mTrackLength; }

protected:
	const FeatureMeasurement *mSourceMeasurement;
	Eigen::Vector2f mPosition;
	const uchar *mDescriptor;
	int mOctave;
	int mTrackLength;
};
//
//class matchingresultsdata
//{
//public:
//	friend class featurematcher;
//
//	matchingresultsdata()		
//	{}
//
//	const keyframe &getframe() const {return *mframe;}
//	void setframe(const keyframe *value) {mframe=value;}
//
//	const eigen::matrix3fr &getpose() const {return mpose;}
//	void setpose(const eigen::matrix3fr &pose) { mpose = pose; }
//
//	featurematch *getmatch(const feature*feature) const
//	{
//		auto it=mmatchedprojectionmap.find(feature);
//		if(it==mmatchedprojectionmap.end())
//			return null;
//		else
//			return it->second;
//	}
//
//	const std::vector<featurematch> &getmatches() const {return mmatches;}
//	std::vector<featurematch> &getmatches() { return mmatches; }
//
//
//	//const matchattempt *getmatchattempt(const slamfeature *feature) const
//	//{
//	//	auto it=mmatchattempts.find(feature);
//	//	if(it==mmatchattempts.end())
//	//		return nullptr;
//	//	else
//	//		return &it->second;
//	//}
//
//	//void creatematchmap();
//
//protected:
//	const keyframe *mframe;
//	eigen::matrix3fr mpose;
//
//	std::vector<featurematch> mmatches;
//};
//
//class featurematcher
//{
//public:
//	featurematcher();
//
//	int getsearchpixeldistance() const {return msearchpixeldistance;}
//	void setsearchdistance(const int pixels)
//	{
//		msearchpixeldistance = pixels;
//		msearchpixeldistancesq = pixels*pixels;
//	}
//
//	int getnonmaximapixelsize() const {return mnonmaximapixelsize;}
//	void setnonmaximapixelsize(int value) {mnonmaximapixelsize=value;}
//
//	int getmaxzssdscore() const {return mmaxzssdscore;}
//	void setmaxzssdscore(int value) {mmaxzssdscore=value;}
//
//	float getbestscorepercentthreshold() const {return mbestscorepercentthreshold;}
//	void setbestscorepercentthreshold(float value) {mbestscorepercentthreshold=value;}
//
//	void setframe(const keyframe *frame);
//	const keyframe &getframe() const { return *mframe; }
//
//	void setpose(const eigen::matrix3fr &pose);
//	const eigen::matrix3fr &getpose() const { return mpose; }
//
//	std::vector<featurematch> &getmatches() { return mmatches; }
//	const std::vector<featurematch> &getmatches() const { return mmatches; }
//
//	void clearresults();
//
//	int findmatches(const std::vector<featureprojectioninfo> &projectionstomatch);
//	featurematch * findmatch(const featureprojectioninfo &projection);
//
//protected:
//	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	// members
//
//	int msearchpixeldistance;
//	int msearchpixeldistancesq;
//
//	struct transformedkeypointposition
//	{
//		eigen::vector2f original;
//		eigen::vector2i transformed;
//
//		//this for the featureindexer
//		eigen::vector2f getposition() const { return original; }
//		int getscore() const { return 0; }
//	};
//	//std::vector<std::vector<transformedkeypointposition>> mtransformedkeypoints;
//	std::vector<featuregridindexer<transformedkeypointposition>> mtransformedkeypoints;
//	
//	const keyframe *mframe;
//
//	eigen::matrix3fr mpose;
//
//	std::vector<featurematch> mmatches;
//
//	int mnonmaximapixelsize;
//	int mmaxzssdscore;
//	float mbestscorepercentthreshold;
//
//
//	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	// methods
//};

}

#endif /* featurematcher_h_ */
