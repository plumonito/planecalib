
#include "Map.h"
#include <map>
#include <cassert>
#include <opencv2/calib3d.hpp>
#include "Keyframe.h"

#include "cvutils.h"
#include "eutils.h"
#include "Profiler.h"

#include "flags.h"
#include "HomographyEstimation.h"

namespace planecalib {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Map
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Map::getFeaturesInView(const Eigen::Matrix3fr &opticalHomography, const CameraModel &camera, const Eigen::Matrix3fr &pose_, int octaveCount, std::unordered_set<const Feature*> &featuresToIgnore, std::vector<std::vector<FeatureProjectionInfo>> &featuresInView)
{
	ProfileSection s("getFeaturesInView");
	
	Eigen::Matrix3fr pose = pose_;
	//pose = Eigen::Matrix3fr::Identity();

	//Find closest keyframe
	HomographyDistance hdist(camera.getImageSize());
	Eigen::Matrix3f hinv = pose.inverse();

	float minDistance = std::numeric_limits<float>::infinity();
	Keyframe *refFrame = NULL;
	//for (auto &framePtr : mKeyframes)
	//{
	//	float dist = hdist.calculateSq(hinv, framePtr->getPose());
	//	if (dist < minDistance)
	//	{
	//		minDistance = dist;
	//		refFrame = framePtr.get();
	//	}
	//}
	refFrame = mKeyframes.begin()->get();


	//Add 3D features
	featuresInView.clear();
	featuresInView.resize(octaveCount);
	for (auto &mPtr : refFrame->getMeasurements())
	{
		Feature &feature = mPtr->getFeature();

		//Ignore?
		if (featuresToIgnore.find(&feature) != featuresToIgnore.end())
			continue;

		//Is the feature inside our image?
		//Eigen::Vector2f pos = eutils::HomographyPoint(pose, feature.getPosition());
		Eigen::Vector2f pos = feature.getPosition();
		if (!eutils::IsInside(camera.getImageSize(), pos))
			continue;

		//Scale
		//Eigen::Vector2f posPlusOne = eutils::HomographyPoint(pose, feature.getPositionPlusOne());
		Eigen::Vector2f posPlusOne = feature.getPositionPlusOne();
		Eigen::Vector2f d = pos - posPlusOne;
		float distSq = std::roundf(d.squaredNorm());
		
		//Scale too small?
		if (distSq < 0.8f)
			continue;

		int octave = 0;
		while (distSq > 1.5f)
		{
			distSq /= 4;
			octave++;
		}

		//Scale too big?
		if (octave > octaveCount - 1 + 0.2f)
			continue;

		int octavei = (int)std::roundf(octave);
		featuresInView[octavei].push_back(FeatureProjectionInfo(&feature, mPtr, octavei, pos));
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
		distSq /= 4;
		octave++;
	}

	return FeatureProjectionInfo(&feature, feature.getMeasurements().back().get(), octave, pos);
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
