
#ifndef CAMERAMODEL_H_
#define CAMERAMODEL_H_

#include <memory>
#include <opencv2/core.hpp>
#include <gflags/gflags.h>
#include "CameraDistortionModel.h"
#include "log.h"

namespace planecalib
{

//Forward
template<class TDistortionModel>
class CameraModel_;

//This typedef decides which camera model to use in the system
//It has to be a typedef because ceres requires templated functions
//to build the automatic diff cost functions and templated methods cannot be virtual.
typedef CameraModel_<DivisionDistortionModel> CameraModel;

template<class TDistortionModel_>
class CameraModel_
{
public:
	CameraModel_()
	{
	}

	typedef TDistortionModel_ TDistortionModel;

	void init(
		const Eigen::Vector2f &principalPoint
		, const Eigen::Vector2f &focalLengths
		, Eigen::Vector2i &imageSize
	) {
		mPrincipalPoint = principalPoint;
		mFocalLengths = focalLengths;
		mImageSize = imageSize;
	}

	const Eigen::Vector2f &getPrincipalPoint() const { return mPrincipalPoint; }
	Eigen::Vector2f &getPrincipalPoint() { return mPrincipalPoint; }
	
	const Eigen::Vector2f &getFocalLength() const { return mFocalLengths; }
	Eigen::Vector2f &getFocalLength() { return mFocalLengths; }
	
	static const int kParamCount=4;
	Eigen::Vector4d getParams() const { return Eigen::Vector4d(mPrincipalPoint[0], mPrincipalPoint[1], mFocalLengths[0], mFocalLengths[1]); }
	void setParams(const Eigen::Vector4d &params) 
	{ 
		mPrincipalPoint[0] = (float)params[0];
		mPrincipalPoint[1] = (float)params[1];
		mFocalLengths[0] = (float)params[2];
		mFocalLengths[1] = (float)params[3];
	}

	const Eigen::Vector2i &getImageSize() const { return mImageSize; }
	TDistortionModel &getDistortionModel() {return mDistortionModel;}
	const TDistortionModel &getDistortionModel() const {return mDistortionModel;}

	void setFromK(const Eigen::Matrix3fr &K)
	{
		mPrincipalPoint = Eigen::Vector2f(K(0, 2), K(1, 2));
		mFocalLengths = Eigen::Vector2f(K(0, 0), K(1, 1));
	}

	bool isPointInside(float depth, const Eigen::Vector2f &p) const
	{
		return depth > 0 &&  //Depth is positive
			p[0] >= 0 && p[1] >= 0 && p[0] < mImageSize[0] && p[1] < mImageSize[1]; //Inside image
	}

	//////////////////////////////////////
	// Project form world
	// Same code, one using return value, one using args by-reference (for ceres)
	Eigen::Vector2f projectFromWorld(const Eigen::Vector3f &xc) const
	{
		const Eigen::Vector2f xn = xc.hnormalized();
		Eigen::Vector2f pn(xn[0] * mFocalLengths[0], xn[1] * mFocalLengths[1]);
		return projectFromScaleSpace(pn);
	}

	//Projects the point assuming focal lengths have already been applied
	Eigen::Vector2f projectFromScaleSpace(const Eigen::Vector2f &pn) const
	{
		return mDistortionModel.apply(pn) + mPrincipalPoint;
	}

	template <class TXMat, class TPMat>
	void projectFromWorld(const Eigen::MatrixBase<TXMat> &x, Eigen::MatrixBase<TPMat> &p) const
	{
		static_assert(TXMat::SizeAtCompileTime == 3, "Param x must be of size 3");
		static_assert(TPMat::SizeAtCompileTime == 2, "Param p must be of size 2");

		typedef typename TXMat::Scalar TScalar;

		Eigen::Matrix<TScalar, 2, 1> xn = x.hnormalized();
		xn[0] *= TScalar(mFocalLengths[0]);
		xn[1] *= TScalar(mFocalLengths[1]);
		mDistortionModel.apply(xn, p);
		p[0] += TScalar(mPrincipalPoint[0]);
		p[1] += TScalar(mPrincipalPoint[1]);
	}

	template <class TPPMat, class TFocalMat, class TDistortionParams, class TXMat, class TPMat>
	static void ProjectFromWorld(const Eigen::MatrixBase<TPPMat> &pp, const Eigen::MatrixBase<TDistortionParams> &distortionParams, const Eigen::MatrixBase<TFocalMat> &focal, const Eigen::MatrixBase<TXMat> &x, Eigen::MatrixBase<TPMat> &p)
	{
		static_assert(TPPMat::SizeAtCompileTime == 2, "Param pp must be of size 2");
		static_assert(TDistortionParams::SizeAtCompileTime == TDistortionModel::TParamVector::SizeAtCompileTime, "Param distortion wrong size");
		static_assert(TFocalMat::SizeAtCompileTime == 2, "Param focal must be of size 2");
		static_assert(TXMat::SizeAtCompileTime == 3, "Param x must be of size 3");
		static_assert(TPMat::SizeAtCompileTime == 2, "Param p must be of size 2");

		typedef typename TXMat::Scalar TScalar;

		Eigen::Matrix<TScalar, 2, 1> xn = x.hnormalized();
		xn[0] *= focal[0];
		xn[1] *= focal[1];
		TDistortionModel::Apply(distortionParams, xn, p);
		p += pp;
	}

	void projectFromWorldJacobian(const Eigen::Vector3f &xc, Eigen::Vector3f &ujac, Eigen::Vector3f &vjac) const;

	//////////////////////////////////////
	// Unproject (assuming z=1)
	Eigen::Vector3f unprojectToWorld(const Eigen::Vector2f &p) const
	{
		Eigen::Vector2f xn = unprojectToScaleSpace(p);

		xn[0] /= mFocalLengths[0];
		xn[1] /= mFocalLengths[1];
		return xn.homogeneous();
	}

	Eigen::Vector2f unprojectToScaleSpace(const Eigen::Vector2f &p) const
	{
		Eigen::Vector2f xd;
		xd[0] = p[0] - mPrincipalPoint[0];
		xd[1] = p[1] - mPrincipalPoint[1];

		return mDistortionModel.applyInv(xd);
	}

	template <class TPMat, class TXMat>
	void unprojectToWorld(const Eigen::MatrixBase<TPMat> &p, Eigen::MatrixBase<TXMat> &x) const
	{
		static_assert(TXMat::SizeAtCompileTime == 3, "Param x must be of size 3");
		static_assert(TPMat::SizeAtCompileTime == 2, "Param p must be of size 2");

		typedef typename TPMat::Scalar TScalar;

		Eigen::Matrix<TScalar, 2, 1> xd;
		xd[0] = p[0] - TScalar(mPrincipalPoint[0]);
		xd[1] = p[1] - TScalar(mPrincipalPoint[1]);
		
		Eigen::Matrix<TScalar, 2, 1> xn;
		mDistortionModel.applyInv(xd, xn);

		xn[0] /= TScalar(mFocalLengths[0]);
		xn[1] /= TScalar(mFocalLengths[1]);
		x = xn.homogeneous();
	}

	template <class TPPMat, class TFocalMat, class TDistortionParams, class TXMat, class TPMat>
	static void UnprojectToWorld(const Eigen::MatrixBase<TPPMat> &pp, const Eigen::MatrixBase<TDistortionParams> &distortionParams, const Eigen::MatrixBase<TFocalMat> &focal, const Eigen::MatrixBase<TXMat> &x, Eigen::MatrixBase<TPMat> &p)
	{
		static_assert(TPPMat::SizeAtCompileTime == 2, "Param pp must be of size 2");
		static_assert(TDistortionParams::SizeAtCompileTime == TDistortionModel::TParamVector::SizeAtCompileTime, "Param distortion wrong size");
		static_assert(TFocalMat::SizeAtCompileTime == 2, "Param focal must be of size 2");
		static_assert(TXMat::SizeAtCompileTime == 3, "Param x must be of size 3");
		static_assert(TPMat::SizeAtCompileTime == 2, "Param p must be of size 2");

		typedef typename TXMat::Scalar TScalar;

		Eigen::Matrix<TScalar, 2, 1> xd;
		xd[0] = p[0] - pp[0];
		xd[1] = p[1] - pp[1];

		Eigen::Matrix<TScalar, 2, 1> xn;
		TDistortionModel::ApplyInv(distortionParams, xd, xn);

		xn[0] /= focal[0];
		xn[1] /= focal[1];
		x = xn.homogeneous();
	}

protected:
	//Four parameters of the intrinsic matrix
	Eigen::Vector2f mPrincipalPoint;
	Eigen::Vector2f mFocalLengths;
	Eigen::Vector2i mImageSize;

	//Distortion model
	TDistortionModel mDistortionModel;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Template implementations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

#endif /* CAMERAMODEL_H_ */
