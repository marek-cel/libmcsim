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
#ifndef MCSIM_AERO_STABILIZERVER_H_
#define MCSIM_AERO_STABILIZERVER_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Vertical stabilizer aerodynamics model class.
 */
class MCSIMAPI StabilizerVer
{
public:

    struct Data
    {
        Vector3 r_ac_bas;          ///< [m] stabilizer aerodynamic center expressed in BAS

        Table cx { Table::oneRecordTable( 0.0 ) };                  ///< [-] drag coefficient vs sideslip angle
        Table cy { Table::oneRecordTable( 0.0 ) };                  ///< [-] sideforce coefficient vs sideslip angle

        double area;               ///< [m^2] stabilizer reference area
    };

    /**
     * @brief Constructor.
     * @param type stabilizer type
     */
    StabilizerVer() = default;

    // LCOV_EXCL_START
    // excluded from coverage report due to deleting destructor calling issues
    /** @brief Destructor. */
    virtual ~StabilizerVer() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param airDensity [kg/m^3] air density
     */
    virtual void computeForceAndMoment( const Vector3 &vel_air_bas,
                                        const Vector3 &omg_air_bas,
                                        double airDensity );

    inline const Vector3& getForce_BAS  () const { return _for_bas; }
    inline const Vector3& getMoment_BAS () const { return _mom_bas; }

protected:

    Data _data;

    Vector3 _for_bas;           ///< [N] total force vector expressed in BAS
    Vector3 _mom_bas;           ///< [N*m] total moment vector expressed in BAS

    /**
     * @brief Computes drag coefficient.
     * @param angle [rad] "angle of attack"
     * @return [-] drag coefficient
     */
    virtual double getCx( double angle ) const;

    /**
     * @brief Computes sideforce coefficient.
     * @param angle [rad] "angle of attack"
     * @return [-] sideforce coefficient
     */
    virtual double getCy( double angle ) const;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_STABILIZERVER_H_
