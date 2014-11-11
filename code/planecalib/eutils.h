/*
 * cvutils.h
 *
 *  Created on: 6.2.2014
 *      Author: dherrera
 */

#ifndef EUTILS_H_
#define EUTILS_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<unsigned char,1,32>);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double, 1, 6 >);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d);

namespace Eigen
{
	typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3fr;
	typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> Matrix4fr;

	typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3dr;

	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdr;
	typedef Eigen::Matrix<float , Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfr;

	typedef Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ArrayXdr;
	typedef Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ArrayXfr;
}

#include <cassert>
#include <opencv2/core/core.hpp>

namespace planecalib
{

class eutils
{
public:
	static bool IsInside(const Eigen::Vector2i &imageSize, const Eigen::Vector2f &p)
	{
		if (p.x() < 0 || p.y() < 0 || p.x() >= imageSize.x() || p.y() >= imageSize.y())
			return false;
		else
			return true;
	}

	static Eigen::Vector2f HomographyPoint(const Eigen::Matrix3f &H, const Eigen::Vector2f &p)
	{
		return (H*p.homogeneous()).eval().hnormalized();
	}

	static Eigen::Vector2i FromSize(const cv::Size2i &sz) 
	{
		return Eigen::Vector2i(sz.width, sz.height);
	}
	static cv::Size2i ToSize(const Eigen::Vector2i &sz)
	{
		return cv::Size2i(sz.x(), sz.y());
	}

	template <class T>
	static Eigen::Matrix<T, 2, 1> FromCV(const cv::Point_<T> &p)
	{
		return Eigen::Matrix<T, 2, 1>(p.x,p.y);
	}

	template <class T, int m, int n>
	static Eigen::Matrix<T, m, n> FromCV(const cv::Matx<T, m, n> &mat)
	{
		Eigen::Matrix<T, m, n> res;
		for (int x = 0; x < n; x++)
			for (int y = 0; y < m; y++)
				res(y, x) = mat(y, x);
		return res;
	}

	template <class T>
	static cv::Point_<T> ToCVPoint(const Eigen::Matrix<T, 2, 1> &p)
	{
		return cv::Point_<T>(p.x(), p.y());
	}

	template <class T, int m, int n, int options>
	static cv::Matx<T,m,n> ToCV(const Eigen::Matrix<T, m, n, options> &emat)
	{
		cv::Matx<T, m, n> cmat;
		Eigen::Map<Eigen::Matrix3fr> cmat_map(cmat.val);
		cmat_map = emat;
		return cmat;
	}
};

}

#endif /* CVUTILS_H_ */
