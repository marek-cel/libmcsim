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
#ifndef LIBMCSIM_AERO_AEROANGLES_H
#define LIBMCSIM_AERO_AEROANGLES_H

////////////////////////////////////////////////////////////////////////////////

#include <mcSim/defs.h>

#include <mcutil/math/Matrix3x3.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Returns angle of attack.
 * @param vel_bas [m/s] airspeed vector
 * @param vel_min [m/s] minimum airspeed of calculations
 * @return [rad] angle of attack
 */
MCSIMEXPORT double getAngleOfAttack( const Vector3 &vel_bas,
                                     double vel_min = 1.0e-2 );

/**
 * @brief Returns angle of attack.
 * @param uv [m/s] airspeed on aircraft xy-plane
 * @param w  [m/s] airspeed along aircraft z-axis
 * @param vel_min [m/s] minimum airspeed of calculations
 * @return [rad] angle of attack
 */
MCSIMEXPORT double getAngleOfAttack( double uv, double w,
                                     double vel_min = 1.0e-2 );

/**
 * @brief Returns sideslip angle.
 * It is positive when the aircraft velocity component along the transverse
 * axis is positive.
 * @see ISO 1151-1:1988
 * @param vel_bas [m/s] airspeed vector
 * @param vel_min [m/s] minimum airspeed of calculations
 * @return [rad] sideslip angle
 */
MCSIMEXPORT double getSideslipAngle( const Vector3 &vel_bas,
                                     double vel_min = 1.0e-2 );

/**
 * @brief Returns rotation matrix from aerodynamic axes system to BAS.
 * @param alpha [rad] angle of attack cosine
 * @param beta [rad] sideslip angle cosine
 * @return rotation matrix from WAS to BAS
 */
MCSIMEXPORT Matrix3x3 getAero2BAS( double alpha, double beta );

/**
 * @brief Returns rotation matrix from aerodynamic axes system to BAS.
 * @param sinAlpha [-] sine of angle of attack cosine
 * @param cosAlpha [-] cosine of angle of attack cosine
 * @param sinBeta  [-] sine of sideslip angle cosine
 * @param cosBeta  [-] cosine of sideslip angle cosine
 * @return rotation matrix from WAS to BAS
 */
MCSIMEXPORT Matrix3x3 getAero2BAS( double sinAlpha , double cosAlpha,
                                   double sinBeta  , double cosBeta );

/**
 * @brief Returns rotation matrix from stability axes system to BAS.
 * @param alpha [rad] angle of attack cosine
 * @return rotation matrix from WAS to BAS
 */
MCSIMEXPORT Matrix3x3 getStab2BAS( double alpha );

/**
 * @brief Returns rotation matrix from stability axes system to BAS.
 * @param sinAlpha [-] sine of angle of attack cosine
 * @param cosAlpha [-] cosine of angle of attack cosine
 * @return rotation matrix from WAS to BAS
 */
MCSIMEXPORT Matrix3x3 getStab2BAS( double sinAlpha, double cosAlpha );

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_AERO_AEROANGLES_H
