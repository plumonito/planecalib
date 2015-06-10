/*
 * HomographyReprojectionError.h
 *
 *  Created on: 29.1.2014
 *      Author: Dan
 */

#ifndef HOMOGRAPHYREPROJECTIONERROR_H_
#define HOMOGRAPHYREPROJECTIONERROR_H_

#include "eutils.h"
#include "PoseEstimationCommon.h"
#include "FeatureMatcher.h"
#include "CameraModel.h"
#include "BaseRansac.h"
#include <opencv2/core.hpp>

namespace planecalib {

class HomographyReprojectionError;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HomographyDistance
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class HomographyDistance
{
public:
	HomographyDistance(const Eigen::Vector2i &imageSize);

	float calculateSq(const Eigen::Matrix3f &h);
	float calculateSq(const Eigen::Matrix3f &h1inv, const Eigen::Matrix3f &h2) { return calculateSq(h1inv*h2); }

protected:
	std::vector<Eigen::Vector2f> mPoints;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HomographyRansac
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct HomographyIterationData
{
	std::vector<MatchReprojectionErrors> reprojectionErrors;
};

class HomographyRansac : public BaseRansac<Eigen::Matrix3dr, HomographyIterationData, 4>
{
public:
	HomographyRansac();
	~HomographyRansac();

	void setData(const std::vector<Eigen::Vector2f> *refPoints, const std::vector<Eigen::Vector2f> *imgPoints, const std::vector<float> *scales);
	void setData(const std::vector<FeatureMeasurement*> &measurements);
	void setData(const std::vector<FeatureMatch> &matches);

	std::vector<Eigen::Matrix3dr> modelFromMinimalSet(const std::vector<int> &constraintIndices);
	void getInliers(const Eigen::Matrix3dr &model, int &inlierCount, float &errorSumSq, HomographyIterationData &data);

protected:
	int mMatchCount;

	const std::vector<Eigen::Vector2f> *mRefPoints;
	const std::vector<Eigen::Vector2f> *mImgPoints;

	std::unique_ptr<std::vector<Eigen::Vector2f>> mOwnRefPoints;
	std::unique_ptr<std::vector<Eigen::Vector2f>> mOwnImgPoints;

	std::vector<std::unique_ptr<HomographyReprojectionError>> mErrorFunctors;
};


class HomographyEstimation
{
public:
	HomographyEstimation(): mMaxIterations(50), mShowIterations(false)
	{
	}

	const int &getMaxIterations() {return mMaxIterations;}
	void setMaxIterations(int value) {mMaxIterations=value;}

	const bool &getShowIterations() {return mShowIterations;}
	void setShowIterations(bool value) {mShowIterations=value;}

	////////////////////////////////////////////////
	// Full homography estimation based on matches
	////////////////////////////////////////////////

	Eigen::Matrix3f estimate(const Eigen::Matrix3f &initial, const std::vector<Eigen::Vector2f> &left, const std::vector<Eigen::Vector2f> &right, const std::vector<float> &scales, float threshold, std::vector<bool> &inliers)
	{
		return estimateCeres(initial, left, right, scales, threshold, inliers);
		//return EstimateOpenCV(initial, left, right, threshold, inliers);
	}
	Eigen::Matrix3fr estimateCeres(const Eigen::Matrix3fr &initial, const std::vector<Eigen::Vector2f> &left, const std::vector<Eigen::Vector2f> &right, const std::vector<float> &scales, float threshold, std::vector<bool> &inliers);
	cv::Matx33f estimateOpenCV(const cv::Matx33f &initial, const std::vector<cv::Point2f> &left, const std::vector<cv::Point2f> &right, const std::vector<int> &octave, float threshold, std::vector<bool> &inliers);

	cv::Matx33f estimateCeres(const cv::Matx33f &initial, const std::vector<cv::Point2f> &left, const std::vector<cv::Point2f> &right, const std::vector<int> &octave, const std::vector<double> &weights, float threshold, std::vector<bool> &inliers);

	////////////////////////////////////////////////
	// Estimation based on direct pixel comparison
	////////////////////////////////////////////////
    /**
     * @brief Estimates a similarity transform that minimizes the SSD between the aligned images.
     * This is the same method as used in PTAM.
     */
    bool estimateSimilarityDirect(const cv::Mat1b &imgRef, const cv::Mat1b &imgNew, cv::Matx23f &transform);
    bool estimateSimilarityDirect(const cv::Mat1b &imgRef, const cv::Mat1s &imgRefDx, const cv::Mat1s &imgRefDy, const cv::Mat1b &imgNew, cv::Matx23f &transform);


	////////////////////////////////////////////////
	// These versions are used for testing only (can be optimized a bit more)
	////////////////////////////////////////////////

    //This version accepts a homography as input
    //The calculated similarity is applied on top of the input transform
    bool estimateSimilarityDirect(const cv::Mat1b &imgRef, const cv::Mat1b &imgNew, Eigen::Matrix3fr &transform);

    bool estimateAffineDirect(const cv::Mat1b &imgRef, const cv::Mat1b &imgNew, cv::Matx33f &transform);

    bool estimateHomographyDirect(const cv::Mat1b &imgRef, const cv::Mat1b &imgNew, cv::Matx33f &transform);

	bool estimateHomographyDirectCeres(const cv::Mat1b &imgRef, const cv::Mat1b &imgNew, Eigen::Matrix3fr &transform);

private:
    int mMaxIterations;
    bool mShowIterations;
};

class HomographyReprojectionError
{
public:
	HomographyReprojectionError(double leftX, double leftY, double rightX, double rightY, double scale)
            : mLeftX(leftX), mLeftY(leftY), mRightX(rightX), mRightY(rightY), mScale(scale)
    {
    }

	//Homography is in row-major order
    template<typename T>
    bool operator()(const T * const homography, T *residuals) const
    {
        T p[3];

        //Translate
        p[0] = homography[0]*T(mRightX) + homography[1]*T(mRightY) + homography[2];
        p[1] = homography[3]*T(mRightX) + homography[4]*T(mRightY) + homography[5];
        p[2] = homography[6]*T(mRightX) + homography[7]*T(mRightY) + homography[8];

        //Normalize
        p[0] /= p[2];
        p[1] /= p[2];

        //Residuals
        residuals[0] = (T(mLeftX) - p[0]) / T(mScale);
        residuals[1] = (T(mLeftY) - p[1]) / T(mScale);
        return true;
    }

	double evalToDistanceSq(const Eigen::Matrix3dr &homography) const
	{
		Eigen::Vector2d residuals;

		(*this)(homography.data(), residuals.data());
		return residuals.squaredNorm();
	}

private:
    const double mLeftX;
    const double mLeftY;
    const double mRightX;
    const double mRightY;
    const double mScale;
};

}

#endif /* HOMOGRAPHYREPROJECTIONERROR_H_ */
