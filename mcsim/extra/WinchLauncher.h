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
#ifndef MCSIM_EXTERNAL_WINCHLAUNCHER_H_
#define MCSIM_EXTERNAL_WINCHLAUNCHER_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Matrix3x3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Sailplane winch launcher model class.
 */
class MCSIMAPI WinchLauncher
{
public:

    struct Data
    {
        Vector3 r_a_bas;            ///< [m] winch cable attachment point expressed in BAS

        double for_max { 0.0 };     ///< [N]   maximum cable force
        double len_max { 0.0 };     ///< [m]   maximum cable length
        double ang_max { 0.0 };     ///< [rad] maximum elevation
        double vel_max { 0.0 };     ///< [m/s] maximum cable velocity

        double tc_for { 0.0 };      ///< [s] force time constant
        double tc_vel { 0.0 };      ///< [s] velocity time constant

        double stiffness { 0.0 };   ///< [N/m^2] cable stiffness
    };

    /** @brief Constructor. */
    WinchLauncher() = default;

    /** @brief Destructor. */
    virtual ~WinchLauncher() = default;

    /**
     * @brief Computes force and moment.
     * @param wgs2bas matrix of rotation from WGS to BAS
     * @param pos_wgs [m] aircraft position expressed in WGS
     */
    virtual void computeForceAndMoment( const Matrix3x3 &wgs2bas,
                                        const Vector3 &pos_wgs );

    /**
     * @brief Update winch model.
     * @param timeStep [s] time step
     * @param bas2wgs matrix of rotation from BAS to WGS
     * @param wgs2ned matrix of rotation from WGS to NED
     * @param pos_wgs [m] aircraft position expressed in WGS
     * @param altitude_agl [m] altitude above ground level
     */
    virtual void update( double timeStep,
                         const Matrix3x3 &bas2wgs,
                         const Matrix3x3 &wgs2ned,
                         const Vector3 &pos_wgs,
                         double altitude_agl );

    inline const Vector3& getForce_BAS  () const { return _for_bas; }
    inline const Vector3& getMoment_BAS () const { return _mom_bas; }

protected:

    Data _data;             ///<

    Vector3 _for_bas;       ///< [N] total force vector expressed in BAS
    Vector3 _mom_bas;       ///< [N*m] total moment vector expressed in BAS

    Vector3 _pos_wgs;       ///< [m] winch position expressed in WGS

    double _for { 0.0 };    ///< [N]   current cable force
    double _vel { 0.0 };    ///< [m/s] current cable velocity
    double _len { 0.0 };    ///< [m]   current cable length

    bool _active { true };  ///< specify if winch is active
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_EXTERNAL_WINCHLAUNCHER_H_
