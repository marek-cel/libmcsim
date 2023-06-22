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
#ifndef MCSIM_AERO_STABILIZERHOR_H_
#define MCSIM_AERO_STABILIZERHOR_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Horizontal stabilizer aerodynamics model class.
 */
class MCSIMAPI StabilizerHor
{
public:

    struct Data
    {
        Vector3 r_ac_bas;           ///< [m] stabilizer aerodynamic center expressed in BAS

        Table cx;                   ///< [-] drag coefficient vs angle of attack
        Table cz;                   ///< [-] lift coefficient vs angle of attack

        Table dw;                   ///< [rad] downwash angle vs wing angle of attack

        double area = 0.0;          ///< [m^2] stabilizer reference area
        double incidence = 0.0;     ///< [rad] stabilizer incidence angle
    };

    /**
     * @brief Constructor.
     * @param type stabilizer type
     */
    StabilizerHor() = default;

    // LCOV_EXCL_START
    // excluded from coverage report due to deleting destructor calling issues
    /** @brief Destructor. */
    virtual ~StabilizerHor() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param airDensity [kg/m^3] air density
     * @param wingAngleOfAttack [rad] wing angle of attack
     */
    virtual void computeForceAndMoment( const Vector3 &vel_air_bas,
                                        const Vector3 &omg_air_bas,
                                        double airDensity,
                                        double wingAngleOfAttack = 0.0 );

    inline const Vector3& getForce_BAS  () const { return _for_bas; }
    inline const Vector3& getMoment_BAS () const { return _mom_bas; }

protected:

    Data _data;

    Vector3 _for_bas;           ///< [N] total force vector expressed in BAS
    Vector3 _mom_bas;           ///< [N*m] total moment vector expressed in BAS

    /**
     * @brief Computes stabilizer angle of attack.
     * @see Etkin B.: Dynamics of Atmosferic Flight, 1972, p.210
     * @see Raymer D.: Aircraft Design: A Conceptual Approach, 1992, p.426
     * @see Paturski Z.: Przewodnik po projektach z Mechaniki Lotu, Projekt nr 9: Rownowaga podluzna samolotu i sily na sterownicy wysokosci, p.IX-4. [in Polish]
     * @param vel_air_bas [m/s] stabilizer linear velocity relative to the air expressed in BAS
     * @param wingAngleOfAttack [rad] wing angle of attack
     * @return [rad] stabilizer angle of attack
     */
    virtual double getAngleOfAttack( const Vector3 &vel_air_bas,
                                     double wingAngleOfAttack );

    /**
     * @brief Computes drag coefficient.
     * @param angle [rad] "angle of attack"
     * @return [-] drag coefficient
     */
    virtual double getCx( double angle ) const;

    /**
     * @brief Computes lift coefficient.
     * @param angle [rad] "angle of attack"
     * @return [-] lift coefficient
     */
    virtual double getCz( double angle ) const;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_STABILIZERHOR_H_
