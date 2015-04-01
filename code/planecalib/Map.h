#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <memory>
#include <atomic>
#include <Eigen/Dense>
#include "eutils.h"
#include "Keyframe.h"
#include "shared_mutex.h"

namespace planecalib {

class Map;
class Keyframe;
class Feature;
class FeatureMeasurement;
class FeatureProjectionInfo;

//This is the top class that holds all data about the world
class Map
{
public:
	Map()
	{
	}

	shared_mutex &getMutex() {return mMutex;}

	void clear();

	const std::vector<std::unique_ptr<Keyframe>> &getKeyframes() const { return mKeyframes; }
	const std::vector<std::unique_ptr<Feature>> &getFeatures() const { return mFeatures; }

	void addKeyframe(std::unique_ptr<Keyframe> newKeyframe);

	void getFeaturesInView(const Eigen::Matrix3fr &pose, const Eigen::Vector2i &imageSize, int octaveCount, std::unordered_set<const Feature*> &featuresToIgnore, std::vector<std::vector<FeatureProjectionInfo>> &featuresInView);
	FeatureProjectionInfo projectFeature(const Eigen::Matrix3fr &pose, Feature &feature);

	Feature *createFeature(Keyframe &keyframe, const Eigen::Matrix3fr &poseInv, const Eigen::Vector2f &position, int octave, const uchar *descriptor);
	void addFeature(std::unique_ptr<Feature> newFeature);

	void moveToGarbage(Feature &feature);

protected:
	shared_mutex mMutex;
	std::vector<std::unique_ptr<Keyframe>> mKeyframes;
	std::vector<std::unique_ptr<Feature>> mFeatures;
	std::vector<std::unique_ptr<Feature>> mGarbageFeatures;
};

class Feature
{
public:
	friend Map;

	const Eigen::Vector2f &getPosition() const {return mPosition;}
	void setPosition(const Eigen::Vector2f &value) {mPosition=value;}

	Eigen::Vector2f getPlusOneOffset() const { return mPlusOneOffset; }
	Eigen::Vector2f getPositionPlusOne() const { return mPosition + mPlusOneOffset; }

	std::vector<std::unique_ptr<FeatureMeasurement>> &getMeasurements() {return mMeasurements;}
	const std::vector<std::unique_ptr<FeatureMeasurement>> &getMeasurements() const {return mMeasurements;}

protected:
	Eigen::Vector2f mPosition;	//Position in world coordinates
	Eigen::Vector2f mPlusOneOffset;	//Relative offset that represents the offset of 1 pixel in the source image.
									//It determines the feature's scale.
									//When (mPosition+mPlusOneOffset) is projected onto the source image it provides
									//the position 1 pixel to the right of the center. This is used to choose the scale where
									//to match the feature.

	std::vector<std::unique_ptr<FeatureMeasurement>> mMeasurements;

public:
	Eigen::Vector3f mPosition3D;
};

class FeatureMeasurement
{
public:
	FeatureMeasurement()
	{}

	FeatureMeasurement(Feature *feature, Keyframe *keyframe, const Eigen::Vector2f &position, int octave, const uchar *descriptor) :
		mFeature(feature), mKeyframe(keyframe), mPosition(position), mOctave(octave)
	{
		Eigen::Map<Eigen::Matrix<uchar, 1, 32>> descMap(const_cast<uchar *>(descriptor));
		mDescriptor = descMap;
	}

	Feature &getFeature() const {return *mFeature;}
	Keyframe &getKeyframe() const {return *mKeyframe;}
	const Eigen::Matrix3fr &getFramePose() const { return mKeyframe->getPose(); } //Shortcut

	Eigen::Vector2f getPosition() const { return mPosition; }
	int getOctave() const {return mOctave;}
	const cv::Mat1b &getImage() const {return mKeyframe->getImage(mOctave);}

	const Eigen::Matrix<uchar,1,32> &getDescriptor() const { return mDescriptor; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	Feature *mFeature;
	Keyframe *mKeyframe;

	Eigen::Vector2f mPosition;
	int mOctave;

	Eigen::Matrix<uchar, 1, 32> mDescriptor;
};

class FeatureProjectionInfo
{
public:
	//////////////////////////
	//Constructors
	FeatureProjectionInfo():
		mFeature(NULL), mSourceMeasurement(NULL)
	{}

	FeatureProjectionInfo(Feature *feature_, FeatureMeasurement *sourceMeasurement_, int octave_, int trackLength_, const Eigen::Vector2f &position) :
		mFeature(feature_), mSourceMeasurement(sourceMeasurement_), mOctave(octave_), mTrackLength(trackLength_), mPosition(position)
	{
	}
	FeatureProjectionInfo(Feature *feature_, FeatureMeasurement *sourceMeasurement_, int octave_, const Eigen::Vector2f &position) :
		mFeature(feature_), mSourceMeasurement(sourceMeasurement_), mOctave(octave_), mTrackLength(0), mPosition(position)
	{
	}

	Feature &getFeature() const {return *mFeature;}
	FeatureMeasurement *getSourceMeasurement() const {return mSourceMeasurement;}
	Eigen::Vector2f getPosition() const { return mPosition; }

	int getOctave() const {return mOctave;}
	int getTrackLength() const {return mTrackLength;}

protected:

	Feature *mFeature;
	FeatureMeasurement *mSourceMeasurement;

	int mOctave;
	int mTrackLength;

	Eigen::Vector2f mPosition;
};

} /* namespace dtslam */

#endif /* SLAMMAP_H_ */
