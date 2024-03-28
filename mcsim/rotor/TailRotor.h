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

#include <cfloat>

#include <mcutils/math/Matrix3x3.h>

#include <mcsim/defs.h>

namespace mc {

/**
 * @brief Helicopter tail rotor model class.
 *
 * \par This model is based on the momentum theory.
 *
 * ### Refernces:
 * - Gessow A Myers G. Aerodynamics of the Helicopter, 1985
 * - Bramwell A. Bramwells Helicopter Dynamics, 2001
 * - Padfield G. Helicopter Flight Dynamics, 2007
 * - Johnson W. Helicopter Theory, 1980
 */
class MCSIMAPI TailRotor
{
public:

    /**
     * @brief Tail rotor data struct
     */
    struct Data
    {
        Vector3 r_hub_bas;              ///< [m]   rotor hub coordinates expressed in BAS
        Angles  a_hub_bas;              ///< [rad] rotor hub orientation relative to BAS

        int nb = 0;                     ///< number of rotor blades

        double blade_mass = 0.0;        ///< [kg] single blade mass

        double r = 0.0;                 ///< [m] rotor radius
        double c = 0.0;                 ///< [m] blades chord

        double a = 0.0;                 ///< [1/rad] blade section lift curve slope
        double b = 0.0;                 ///< [-] tip losses coefficient

        double delta_0 = 0.0;           ///< [-] drag coefficient constant component
        double delta_2 = 0.0;           ///< [-] drag coefficient quadratic component

        double ct_max = DBL_MAX;        ///< [-] maximum thrust coefficient
        double cq_max = DBL_MAX;        ///< [-] maximum torque coefficient

        double thrust_factor = 1.0;     ///< [-] thrust scaling factor
        double torque_factor = 1.0;     ///< [-] torque scaling factor
    };

    virtual const Data* GetData() const = 0;

    /**
     * @brief Updates force and moment.
     * @param vel_air_bas [m/s]    aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s]  aircraft angular velocity relative to the air expressed in BAS
     * @param airDensity  [kg/m^3] air density
     */
    virtual void UpdateForceAndMoment(const Vector3& vel_air_bas,
                                      const Vector3& omg_air_bas,
                                      double airDensity);

    /**
     * @brief Updates rotor model.
     * @param omega      [rad/s] rotor revolution speed
     * @param collective [rad]   collective pitch angle
     */
    virtual void Update(double omega, double collective);

    inline const Vector3& f_bas() const { return f_bas_; }
    inline const Vector3& m_bas() const { return m_bas_; }

    inline double ir() const { return ir_; }

    inline double thrust() const { return thrust_; }
    inline double torque() const { return torque_; }

    inline double lambda()    const { return lambda_;    }
    inline double lambda_i()  const { return lambda_i_;  }
    inline double lambda_i0() const { return lambda_i0_; }

    inline double vel_i()  const { return vel_i_;  }
    inline double vel_i0() const { return vel_i0_; }

protected:

    Vector3 f_bas_;             ///< [N] total force vector expressed in BAS
    Vector3 m_bas_;             ///< [N*m] total moment vector expressed in BAS

    Matrix3x3 bas2ras_;         ///< matrix of rotation from BAS to RAS
    Matrix3x3 ras2bas_;         ///< matrix of rotation from RAS to BAS

    double r2_ = 0.0;           ///< [m^2] rotor radius squared
    double r3_ = 0.0;           ///< [m^3] rotor radius cubed
    double ar_ = 0.0;           ///< [m^2] rotor disk area
    double s_  = 0.0;           ///< [-]   rotor solidity

    double ib_ = 0.0;           ///< [kg*m^2] single rotor blade inertia moment about flapping hinge
    double ir_ = 0.0;           ///< [kg*m^2] rotor total inertia about shaft axis

    double omega_ = 0.0;        ///< [rad/s]     rotor revolution speed
    double omega2_ = 0.0;       ///< [rad^2/s^2] rotor revolution speed squared
    double omegaR_ = 0.0;       ///< [m/s]       rotor tip velocity

    double theta_ = 0.0;        ///< [rad] feathering angle

    double thrust_ = 0.0;       ///< [N]   rotor thrust
    double torque_ = 0.0;       ///< [N*m] rotor torque

    double lambda_    = 0.0;    ///< [-] normalized velocity at rotor disc
    double lambda_i_  = 0.0;    ///< [-] normalized rotor induced velocity
    double lambda_i0_ = 0.0;    ///< [-] normalized rotor induced velocity in hover

    double vel_i_ = 0.0;        ///< [m/s] rotor induced velocity
    double vel_i0_ = 0.0;       ///< [m/s] rotor induced velocity in hover

    unsigned int n_max_ = 100;  ///< maximum number of iteration loop steps

    /**
     * @brief Updates auxiliary variables that are directly derived from data.
     * Examples of such variables: radius squared, disk area, solidity, etc.
     */
    virtual void UpdateDataDerivedVariables();
};

} // namespace mc

#endif // MCSIM_ROTOR_TAILROTOR_H_
