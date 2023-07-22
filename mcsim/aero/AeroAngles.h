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
#ifndef MCSIM_AERO_AEROANGLES_H_
#define MCSIM_AERO_AEROANGLES_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Matrix3x3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Returns angle of attack.
 *
 * ### Refernces:
 * - International Standard: Flight dynamics - Concepts, quantities and symbols - Part 1: Aircraft motion relative to the air. ISO 1151-1:1988
 * - Drela M.: Flight Vehicle Aerodynamics. 2014. p.203
 *
 * @param u [m/s] airspeed along aircraft x-axis
 * @param w [m/s] airspeed along aircraft z-axis
 * @param vel_min [m/s] minimum airspeed of calculations
 * @return [rad] angle of attack
 */
MCSIMAPI double GetAngleOfAttack(double u, double w,
                                 double vel_min = 1.0e-6);

/**
 * @brief Returns angle of attack.
 * @param vel_bas [m/s] airspeed vector
 * @param vel_min [m/s] minimum airspeed of calculations
 * @return [rad] angle of attack
 */
MCSIMAPI inline double GetAngleOfAttack(const Vector3& vel_bas,
                                        double vel_min = 1.0e-6)
{
    return GetAngleOfAttack(vel_bas.u(), vel_bas.w(), vel_min);
}

/**
 * @brief Returns sideslip angle.
 *
 * \par It is positive when the aircraft velocity component along the transverse
 * axis is positive.
 *
 * ### Refernces:
 * - International Standard: Flight dynamics - Concepts, quantities and symbols - Part 1: Aircraft motion relative to the air. ISO 1151-1:1988
 * - Drela M.: Flight Vehicle Aerodynamics. 2014. p.203
 *
 * @param v [m/s] airspeed along aircraft y-axis
 * @param uw [m/s] airspeed projected on the x-z plane
 * @param vel_min [m/s] minimum airspeed of calculations
 * @return [rad] sideslip angle
 */
MCSIMAPI double GetSideslipAngle(double v, double uw,
                                 double vel_min = 1.0e-6);

/**
 * @brief Returns sideslip angle.
 * It is positive when the aircraft velocity component along the transverse
 * axis is positive.
 * @param vel_bas [m/s] airspeed vector
 * @param vel_min [m/s] minimum airspeed of calculations
 * @return [rad] sideslip angle
 */
MCSIMAPI inline double GetSideslipAngle(const Vector3& vel_bas,
                                        double vel_min = 1.0e-6)
{
    return GetSideslipAngle(vel_bas.v(), vel_bas.GetLengthXZ(), vel_min);
}

/**
 * @brief Returns rotation matrix from aerodynamic axes system to BAS.
 * @param alpha [rad] angle of attack
 * @param beta [rad] sideslip angle
 * @return rotation matrix from WAS to BAS
 */
MCSIMAPI Matrix3x3 GetAero2BAS(double alpha, double beta);

/**
 * @brief Returns rotation matrix from aerodynamic axes system to BAS.
 * @param sin_alpha [-] angle of attack sine
 * @param cos_alpha [-] angle of attack cosine
 * @param sin_beta  [-] sideslip angle sine
 * @param cos_beta  [-] sideslip angle cosine
 * @return rotation matrix from WAS to BAS
 */
MCSIMAPI Matrix3x3 GetAero2BAS(double sin_alpha , double cos_alpha,
                               double sin_beta  , double cos_beta);

/**
 * @brief Returns rotation matrix from stability axes system to BAS.
 * @param alpha [rad] angle of attack
 * @return rotation matrix from WAS to BAS
 */
MCSIMAPI Matrix3x3 GetStab2BAS(double alpha);

/**
 * @brief Returns rotation matrix from stability axes system to BAS.
 * @param sin_alpha [-] angle of attack sine
 * @param cos_alpha [-] angle of attack cosine
 * @return rotation matrix from WAS to BAS
 */
MCSIMAPI Matrix3x3 GetStab2BAS(double sin_alpha, double cos_alpha);

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_AEROANGLES_H_
