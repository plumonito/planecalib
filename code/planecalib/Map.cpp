
#include "Map.h"
#include <map>
#include <cassert>
#include <opencv2/calib3d.hpp>
#include "Keyframe.h"

#include "cvutils.h"
#include "eutils.h"
#include "Profiler.h"

#include "flags.h"

namespace planecalib {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Map
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Map::getFeaturesInView(const Eigen::Matrix3fr &pose, const Eigen::Vector2i &imageSize, int octaveCount, std::unordered_set<const Feature*> &featuresToIgnore, std::vector<std::vector<FeatureProjectionInfo>> &featuresInView)
{
	ProfileSection s("getFeaturesInView");

	featuresInView.clear();
	featuresInView.resize(octaveCount);

	//Add 3D features
	for (auto &pfeature : mFeatures)
	{
		Feature &feature = *pfeature;

		//Ignore?
		if (featuresToIgnore.find(&feature) != featuresToIgnore.end())
			continue;

		FeatureProjectionInfo projection = projectFeature(pose, feature);
		if (projection.getSourceMeasurement() && projection.getOctave() < octaveCount && eutils::IsInside(imageSize, projection.getPosition()))
		{
			featuresInView[projection.getOctave()].push_back(projection);
		}
	}
}

FeatureProjectionInfo Map::projectFeature(const Eigen::Matrix3fr &pose, Feature &feature)
{
	Eigen::Vector2f pos = eutils::HomographyPoint(pose, feature.getPosition());
	Eigen::Vector2f posPlusOne = eutils::HomographyPoint(pose, feature.getPositionPlusOne());
	
	Eigen::Vector2f d = pos - posPlusOne;
	int distSq = (int)std::roundf(d.squaredNorm());

	if (distSq < 1)
		return FeatureProjectionInfo();

	int octave = 0;
	while (distSq > 1)
	{
		distSq /= 2;
		octave++;
	}

	return FeatureProjectionInfo(&feature, feature.getMeasurements()[0].get(), octave, pos);
}

void Map::addKeyframe(std::unique_ptr<Keyframe> newKeyframe)
{
	mKeyframes.push_back(std::move(newKeyframe));
}

void Map::addFeature(std::unique_ptr<Feature> newFeature)
{
	mFeatures.push_back(std::move(newFeature));
}

Feature *Map::createFeature(Keyframe &keyframe, const Eigen::Matrix3fr &poseInv, const Eigen::Vector2f &position, int octave, const uchar *descriptor)
{
	int scale = 1 << octave;

	//Create feature
	std::unique_ptr<Feature> feature(new Feature());

	Eigen::Vector2f pos = eutils::HomographyPoint(poseInv, position);
	Eigen::Vector2f posPlusOne = eutils::HomographyPoint(poseInv, position+Eigen::Vector2f(scale,0));
	
	feature->mPosition = pos;
	feature->mPlusOneOffset = posPlusOne - pos;
	
	//Create measurement
	feature->mMeasurements.emplace_back(new FeatureMeasurement(feature.get(), &keyframe, position, octave, descriptor));
	keyframe.getMeasurements().push_back(feature->mMeasurements.back().get());

	//Add feature to map
	Feature *pfeature = feature.get();
	mFeatures.push_back(std::move(feature));

	return pfeature;
}

}
