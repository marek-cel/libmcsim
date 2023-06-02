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
#ifndef MCSIM_AERO_FUSELAGE_H_
#define MCSIM_AERO_FUSELAGE_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Fuselage aerodynamics model class.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li><a href="https://ntrs.nasa.gov/citations/19830001781">A Mathematical Model of a Single Main Rototr Helicopter for Piloted Simulation. NASA-TM-84281</a></li>
 * </ul>
 */
class MCSIMAPI Fuselage
{
public:

    struct Data
    {
        Vector3 r_ac_bas;                           ///< [m] fuselage aerodynamic center expressed in BAS

        Table cx { Table::oneRecordTable( 0.0 ) };  ///< [-] drag coefficient vs [rad] angle of attack
        Table cy { Table::oneRecordTable( 0.0 ) };  ///< [-] sideforce coefficient vs [rad] angle of sideslip
        Table cz { Table::oneRecordTable( 0.0 ) };  ///< [-] lift coefficient vs [rad] angle of attack

        Table cl { Table::oneRecordTable( 0.0 ) };  ///< [-] rolling moment coefficient vs [rad] angle of sideslip
        Table cm { Table::oneRecordTable( 0.0 ) };  ///< [-] pitching moment coefficient vs [rad] angle of attack
        Table cn { Table::oneRecordTable( 0.0 ) };  ///< [-] yawing moment coefficient vs [rad] angle of sideslip

        double length { 0.0 } ;                     ///< [m] reference length
        double area   { 0.0 } ;                     ///< [m^2] reference area
    };

    /** @brief Constructor. */
    Fuselage() = default;

    // LCOV_EXCL_START
    // excluded from coverage report due to deleting destructor calling issues
    /** @brief Destructor. */
    virtual ~Fuselage() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param airDensity [kg/m^3] air density
     * @param inducedVelocity [m/s] rotor induced velocity
     * @param wakeSkewAngle [rad] rotor wake skew angle
     */
    virtual void computeForceAndMoment( const Vector3& vel_air_bas,
                                        const Vector3& omg_air_bas,
                                        double airDensity,
                                        double inducedVelocity = 0.0,
                                        double wakeSkewAngle = 0.0 );

    inline const Vector3& getForce_BAS  () const { return for_bas_; }
    inline const Vector3& getMoment_BAS () const { return mom_bas_; }

protected:

    Data data_;

    Vector3 for_bas_;           ///< [N] total force vector expressed in BAS
    Vector3 mom_bas_;           ///< [N*m] total moment vector expressed in BAS

    double area_length_;        ///< [m^3] S*l where S is reference area and l is reference length

    double angleOfAttack_;      ///< [rad] angle of attack
    double sideslipAngle_;      ///< [rad] angle of sideslip

    /**
     * @brief Computes drag coefficient.
     * @param angleOfAttack [rad] angle of attack
     * @return [-] drag coefficient
     */
    virtual double getCx( double angleOfAttack ) const;

    /**
     * @brief Computes sideforce coefficient.
     * @param sideslipAngle [rad] angle of sideslip
     * @return [-] sideforce coefficient
     */
    virtual double getCy( double sideslipAngle ) const;

    /**
     * @brief Computes lift coefficient.
     * @param angleOfAttack [rad] angle of attack
     * @return [-] lift coefficient
     */
    virtual double getCz( double angleOfAttack ) const;

    /**
     * @brief Computes rolling moment coefficient.
     * @param sideslipAngle [rad] angle of sideslip
     * @return [-] rolling moment coefficient
     */
    virtual double getCl( double sideslipAngle ) const;

    /**
     * @brief Computes pitching moment coefficient.
     * @param angleOfAttack [rad] angle of attack
     * @return [-] pitching moment coefficient
     */
    virtual double getCm( double angleOfAttack ) const;

    /**
     * @brief Computes yawing moment coefficient.
     * @param sideslipAngle [rad] angle of sideslip
     * @return [-] yawing moment coefficient
     */
    virtual double getCn( double sideslipAngle ) const;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_FUSELAGE_H_
