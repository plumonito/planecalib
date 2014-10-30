
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

	void Map::getFeaturesInView(const Eigen::Matrix3f &pose, const Eigen::Vector2i &imageSize, int octaveCount, std::unordered_set<Feature*> &featuresToIgnore, std::vector<std::vector<FeatureProjectionInfo>> &featuresInView)
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

FeatureProjectionInfo Map::projectFeature(const Eigen::Matrix3f &pose, Feature &feature)
{
	Eigen::Vector2f pos = eutils::HomographyPoint(pose, feature.getPosition());
	Eigen::Vector2f posPlusOne = eutils::HomographyPoint(pose, feature.getPositionPlusOne());
	
	Eigen::Vector2f d = pos - posPlusOne;
	int distSq = (int)std::roundf(d.squaredNorm());

	if (distSq < 1)
		return FeatureProjectionInfo();

	int octave = 0;
	while (distSq > 0)
	{
		distSq /= 2;
		octave++;
	}

	return FeatureProjectionInfo(&feature, feature.getMeasurements()[0].get(), octave, pos);
}

}
