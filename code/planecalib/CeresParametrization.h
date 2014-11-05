//----------------------------------------------------------------------------------
//
// Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//
//----------------------------------------------------------------------------------

#ifndef FIXED3DNORMPARAMETRIZATION_H_
#define FIXED3DNORMPARAMETRIZATION_H_

#include <ceres/local_parameterization.h>
#include "CeresUtils.h"

namespace planecalib
{

/**
 * @brief A parameterization class that is used for CERES solver. It parametrizes the translation of a pose with two components, keeping the L2 norm fixed
 */
class Fixed3DNormParametrization: public ceres::LocalParameterization
{
public:
    Fixed3DNormParametrization(double norm)
            : mFixedNorm(norm)
    {
    }
    virtual ~Fixed3DNormParametrization()
    {
    }

    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const
    {
        return 3;
    }
    virtual int LocalSize() const
    {
        return 2;
    }

    /**
     * @brief Calculates two vectors that are orthogonal to X. It first picks a non-colinear point C then basis1=(X-C) x C and basis2=X x basis1
     */
    static void GetBasis(const double *x, double *basis1, double *basis2);

protected:
    const double mFixedNorm;
};

/**
 * @brief A parameterization class that is used for CERES solver. It parametrizes a 4D vector with three components, keeping the L2 norm fixed
 */
class Fixed4DNormParametrization: public ceres::LocalParameterization
{
public:
    Fixed4DNormParametrization(double norm)
            : mFixedNorm(norm)
    {
    }
    virtual ~Fixed4DNormParametrization()
    {
    }

    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const
    {
        return 4;
    }
    virtual int LocalSize() const
    {
        return 3;
    }

    /**
     * @brief Calculates 3 vectors that are orthogonal to X.
     */
    static void GetBasis(const double *x, double *basis1, double *basis2, double *basis3);

protected:
    const double mFixedNorm;
};

/*
 * I think specifying the full class (as above) is faster
struct Fixed4DNormPlus
{
	template<typename T>
	bool operator()(const T* x, const T* delta, T* x_plus_delta) const
	{
		double xd[4];
	    double basis1[4];
	    double basis2[4];
	    double basis3[4];

	    //To double
	    xd[0] = CeresUtils::ToDouble(x[0]);
	    xd[1] = CeresUtils::ToDouble(x[1]);
	    xd[2] = CeresUtils::ToDouble(x[2]);
	    xd[3] = CeresUtils::ToDouble(x[3]);

	    //Translation is constrained
	    Fixed4DNormParametrization::GetBasis(xd, basis1, basis2, basis3);

	    x_plus_delta[0] = x[0] + delta[0] * T(basis1[0]) + delta[1] * T(basis2[0]) + delta[2] * T(basis3[0]);
	    x_plus_delta[1] = x[1] + delta[0] * T(basis1[1]) + delta[1] * T(basis2[1]) + delta[2] * T(basis3[1]);
	    x_plus_delta[2] = x[2] + delta[0] * T(basis1[2]) + delta[1] * T(basis2[2]) + delta[2] * T(basis3[2]);
	    x_plus_delta[3] = x[3] + delta[0] * T(basis1[3]) + delta[1] * T(basis2[3]) + delta[2] * T(basis3[3]);

	    T norm = ceres::sqrt(
	            x_plus_delta[0] * x_plus_delta[0] + x_plus_delta[1] * x_plus_delta[1] + x_plus_delta[2] * x_plus_delta[2]
	            + x_plus_delta[3] * x_plus_delta[3]);
	    T factor = T(1) / norm;
	    x_plus_delta[0] *= factor;
	    x_plus_delta[1] *= factor;
	    x_plus_delta[2] *= factor;
	    x_plus_delta[3] *= factor;

	    return true;
	}
};
*/
}

#endif /* FIXED3DNORMPARAMETRIZATION_H_ */
