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

double getAngleOfAttack( double u, double w, double vel_min )
{
    double angleOfAttack = 0.0;

    if ( fabs( u ) > vel_min || fabs( w ) > vel_min )
    {
        // Drela M.: Flight Vehicle Aerodynamics. p.203
        angleOfAttack = atan2( w, u );
    }

    return angleOfAttack;
}

////////////////////////////////////////////////////////////////////////////////

double getSideslipAngle( double v, double uw, double vel_min )
{
    double sideslipAngle = 0.0;

    double uw_abs = fabs( uw );

    if ( uw_abs > vel_min || fabs( v ) > vel_min )
    {
        // Drela M.: Flight Vehicle Aerodynamics. p.203
        sideslipAngle = atan2( v, uw_abs );
    }

    return sideslipAngle;
}

////////////////////////////////////////////////////////////////////////////////

Matrix3x3 getAero2BAS( double alpha, double beta )
{
    return getAero2BAS( sin( alpha ), cos( alpha ),
                        sin( beta ), cos( beta ) );
}

////////////////////////////////////////////////////////////////////////////////

Matrix3x3 getAero2BAS( double sinAlpha , double cosAlpha,
                       double sinBeta  , double cosBeta )
{
    Matrix3x3 aero2bas;

    aero2bas(0,0) = -cosAlpha * cosBeta;
    aero2bas(0,1) = -cosAlpha * sinBeta;
    aero2bas(0,2) =  sinAlpha;

    aero2bas(1,0) = -sinBeta;
    aero2bas(1,1) =  cosBeta;
    aero2bas(1,2) =  0.0;

    aero2bas(2,0) = -sinAlpha * cosBeta;
    aero2bas(2,1) = -sinAlpha * sinBeta;
    aero2bas(2,2) = -cosAlpha;

    return aero2bas;
}

////////////////////////////////////////////////////////////////////////////////

Matrix3x3 getStab2BAS( double alpha )
{
    return getStab2BAS( sin( alpha ), cos( alpha ) );
}

////////////////////////////////////////////////////////////////////////////////

Matrix3x3 getStab2BAS( double sinAlpha, double cosAlpha )
{
    Matrix3x3 stab2bas;

    stab2bas(0,0) = -cosAlpha;
    stab2bas(0,1) =  0.0;
    stab2bas(0,2) =  sinAlpha;

    stab2bas(1,0) = 0.0;
    stab2bas(1,1) = 1.0;
    stab2bas(1,2) = 0.0;

    stab2bas(2,0) = -sinAlpha;
    stab2bas(2,1) =  0.0;
    stab2bas(2,2) = -cosAlpha;

    return stab2bas;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
