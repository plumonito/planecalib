#ifndef CAMERAMODELCERES_H_
#define CAMERAMODELCERES_H_

#include "CameraModel.h"
#include <ceres/ceres.h>
//#include <ceres/jet.h>

namespace planecalib
{

template<class TDistortionModel>
void CameraModel_<TDistortionModel>::projectFromWorldJacobian(const Eigen::Vector3f &xc, Eigen::Vector3f &ujac, Eigen::Vector3f &vjac) const
{
	typedef ceres::Jet<double, 3> TJet;
	Eigen::Matrix<TJet, 3, 1> x(TJet(xc[0], 0), TJet(xc[1], 1), TJet(xc[2], 2));

	Eigen::Matrix<TJet, 2, 1> p;

	projectFromWorld(x,p);
	ujac[0] = (float)p[0].v[0];
	ujac[1] = (float)p[0].v[1];
	ujac[2] = (float)p[0].v[2];

	vjac[0] = (float)p[1].v[0];
	vjac[1] = (float)p[1].v[1];
	vjac[2] = (float)p[1].v[2];
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// ForwardDistortionFunction & InverseDistortionFunction

template <class TDistortionModel>
class ForwardDistortionFunction : public ceres::SizedCostFunction<2, TDistortionModel::TParamVector::SizeAtCompileTime, 2>
{
public:
	typedef ceres::CostFunctionToFunctor<2, TDistortionModel::TParamVector::SizeAtCompileTime, 2> TCostFunctor;

	bool Evaluate(double const * const * parameters, double *output, double**jacobians) const
	{
		Eigen::Map<TDistortionModel::TParamVector> params(parameters[0]);
		Eigen::Map<Eigen::Vector2d> x(parameters[1]);
		Eigen::Map<Eigen::Vector2d> xd(output);

		TDistortionModel::EvaluateCeres(params, x, xd, jacobians);
	}
};

template <class TDistortionModel>
class InverseDistortionFunction : public ceres::SizedCostFunction<2, TDistortionModel::TParamVector::SizeAtCompileTime, 2>
{
public:
	typedef ceres::CostFunctionToFunctor<2, TDistortionModel::TParamVector::RowsAtCompileTime, 2> TCostFunctor;

	bool Evaluate(double const * const * parameters, double *output, double**jacobians) const
	{
		Eigen::Map<TDistortionModel::TParamVector> params(parameters[0]);
		Eigen::Map<Eigen::Vector2d> xd(parameters[1]);
		Eigen::Map<Eigen::Vector2d> x(output);

		TDistortionModel::EvaluateInvCeres(params, x, xd, jacobians);
	}
};

template class ForwardDistortionFunction<DivisionDistortionModel>;
template class InverseDistortionFunction<DivisionDistortionModel>;

}

#endif /* CAMERAMODELCERES_H_ */
