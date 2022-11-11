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
#ifndef MCSIM_ROTOR_TAILROTOR_H_
#define MCSIM_ROTOR_TAILROTOR_H_

////////////////////////////////////////////////////////////////////////////////

#include <cfloat>

#include <mcutils/math/Matrix3x3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Helicopter tail rotor model class.
 *
 * This model is based on the momentum theory.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Gessow A Myers G. Aerodynamics of the Helicopter, 1985</li>
 *   <li>Bramwell A. Bramwells Helicopter Dynamics, 2001</li>
 *   <li>Padfield G. Helicopter Flight Dynamics, 2007</li>
 *   <li>Johnson W. Helicopter Theory, 1980</li>
 * </ul>
 */
class MCSIMAPI TailRotor
{
public:

    /**
     * @brief Tail rotor data struct
     */
    struct Data
    {
        Vector3 r_hub_bas;              ///< [m] rotor hub coordinates expressed in BAS

        int nb { 0 };                   ///< number of rotor blades

        double r { 0.0 };               ///< [m] rotor radius
        double c { 0.0 };               ///< [m] blades chord

        double a { 0.0 };               ///< [1/rad] blade section lift curve slope
        double b { 0.0 };               ///< [-] tip losses coefficient

        double delta_0 { 0.0 };         ///< [-] drag coefficient constant component
        double delta_2 { 0.0 };         ///< [-] drag coefficient quadratic component

        double ct_max { DBL_MAX };      ///< [-] maximum thrust coefficient
        double cq_max { DBL_MAX };      ///< [-] maximum torque coefficient

        double thrust_factor { 1.0 };   ///< [-] thrust scaling factor
        double torque_factor { 1.0 };   ///< [-] torque scaling factor
    };

    /** @brief Constructor. */
    TailRotor() = default;

    /** @brief Destructor. */
    virtual ~TailRotor() = default;

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s]    aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s]  aircraft angular velocity relative to the air expressed in BAS
     * @param airDensity  [kg/m^3] air density
     */
    virtual void computeForceAndMoment( const Vector3 &vel_air_bas,
                                        const Vector3 &omg_air_bas,
                                        double airDensity );

    /**
     * @brief Updates rotor model.
     * @param omega      [rad/s] rotor revolution speed
     * @param collective [rad]   collective pitch angle
     */
    virtual void update( double omega, double collective );

    inline const Vector3& getForce_BAS  () const { return _for_bas; }
    inline const Vector3& getMoment_BAS () const { return _mom_bas; }

    inline double getRotorInertia() const { return _ir; }

    inline double getThrust() const { return _thrust; }
    inline double getTorque() const { return _torque; }

protected:

    Data _data;                 ///< tail rotor data

    Vector3 _for_bas;           ///< [N] total force vector expressed in BAS
    Vector3 _mom_bas;           ///< [N*m] total moment vector expressed in BAS

    Matrix3x3 _bas2ras;         ///< matrix of rotation from BAS to RAS
    Matrix3x3 _ras2bas;         ///< matrix of rotation from RAS to BAS

    double _r2 { 0.0 };         ///< [m^2] rotor radius squared
    double _r3 { 0.0 };         ///< [m^3] rotor radius cubed
    double _ar { 0.0 };         ///< [m^2] rotor disk area
    double _s  { 0.0 };         ///< [-]   rotor solidity

    double _ib { 0.0 };         ///< [kg*m^2] single rotor blade inertia moment about flapping hinge
    double _ir { 0.0 };         ///< [kg*m^2] rotor total inertia about shaft axis

    double _omega { 0.0 };      ///< [rad/s] rotor revolution speed

    double _theta { 0.0 };      ///< [rad] feathering angle

    double _thrust { 0.0 };     ///< [N]   rotor thrust
    double _torque { 0.0 };     ///< [N*m] rotor torque

    double _vel_i { 0.0 };      ///< [m/s] rotor induced velocity
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_ROTOR_TAILROTOR_H_
