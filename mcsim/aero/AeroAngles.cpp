/****************************************************************************//*
 * Copyright (C) 2022 Marek M. Cel
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#include <mcsim/aero/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

double GetAngleOfAttack(double u, double w, double vel_min)
{
    double alpha = 0.0;
    if ( fabs(u) > vel_min || fabs(w) > vel_min )
    {
        // Drela M.: Flight Vehicle Aerodynamics. p.203
        alpha = atan2(w, u);
    }
    return alpha;
}

////////////////////////////////////////////////////////////////////////////////

double GetSideslipAngle(double v, double uw, double vel_min)
{
    double beta = 0.0;
    double uw_abs = fabs(uw);
    if ( uw_abs > vel_min || fabs(v) > vel_min )
    {
        // Drela M.: Flight Vehicle Aerodynamics. p.203
        beta = atan2(v, uw_abs);
    }
    return beta;
}

////////////////////////////////////////////////////////////////////////////////

Matrix3x3 GetAero2BAS(double alpha, double beta)
{
    return GetAero2BAS(sin(alpha), cos(alpha), sin(beta), cos(beta));
}

////////////////////////////////////////////////////////////////////////////////

Matrix3x3 GetAero2BAS(double sin_alpha , double cos_alpha,
                      double sin_beta  , double cos_beta)
{
    Matrix3x3 aero2bas;

    aero2bas(0,0) = -cos_alpha * cos_beta;
    aero2bas(0,1) = -cos_alpha * sin_beta;
    aero2bas(0,2) =  sin_alpha;

    aero2bas(1,0) = -sin_beta;
    aero2bas(1,1) =  cos_beta;
    aero2bas(1,2) =  0.0;

    aero2bas(2,0) = -sin_alpha * cos_beta;
    aero2bas(2,1) = -sin_alpha * sin_beta;
    aero2bas(2,2) = -cos_alpha;

    return aero2bas;
}

////////////////////////////////////////////////////////////////////////////////

Matrix3x3 GetStab2BAS(double alpha)
{
    return GetStab2BAS(sin(alpha), cos(alpha));
}

////////////////////////////////////////////////////////////////////////////////

Matrix3x3 GetStab2BAS(double sin_alpha, double cos_alpha)
{
    Matrix3x3 stab2bas;

    stab2bas(0,0) = -cos_alpha;
    stab2bas(0,1) =  0.0;
    stab2bas(0,2) =  sin_alpha;

    stab2bas(1,0) = 0.0;
    stab2bas(1,1) = 1.0;
    stab2bas(1,2) = 0.0;

    stab2bas(2,0) = -sin_alpha;
    stab2bas(2,1) =  0.0;
    stab2bas(2,2) = -cos_alpha;

    return stab2bas;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
