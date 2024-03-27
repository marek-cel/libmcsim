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
#ifndef MCSIM_ROTOR_MAINROTOR_H_
#define MCSIM_ROTOR_MAINROTOR_H_

#include <cfloat>
#include <memory>

#include <mcutils/math/Angles.h>
#include <mcutils/math/Matrix3x3.h>

#include <mcsim/defs.h>

namespace mc {

/**
 * @brief Helicopter main rotor model class.
 *
 * @par This model combines induced velocity calculated using momentum theory
 * and flapping equations developed analytically using blade element theory.
 *
 * @par Flapping angle is positive upwards.
 *
 * ### Coordinate Systems Used for Rotor Calculations
 *
 * @par Rotor Axis System (RAS)
 * Origin of the Rotor Axis System is coincident with the rotor hub center,
 * the x-axis is positive forwards, the y-axis is positive right and z-axis
 * is positive downwards and coincident with the rotor shaft axis.
 *
 * @par Rotor-Wind Axis System (RWAS)
 * Rotor-Wind Axis System is very much like Rotor Axis System, the only
 * difference is that it is rotated about z-axis in such a manner that x-axis
 * points directly into relative wind, so there is no lateral airspeed
 * component.
 *
 * @par Control Axis System (CAS)
 * For most purposes, using the Rotor Axis System causes unnecessary
 * complications. It is convenient to use no cyclic feathering axis system.
 * Origin of the Control Axis System is coincident with the origin of the Rotor
 * Axis System, but it is rotated by angles of the swashplate roll and pitch so
 * there is no cyclic feathering in this coordinate system.
 *
 * @par Disc Axis System (DAS)
 * Origin of the Disc Axis System is coincident with the origin of the Rotor
 * Axis System, but it is rotated by angles of the rotor cone roll and pitch
 * in such a manner that z?axis is perpendicular to the tip path plane so there
 * is no cyclic flapping in this coordinate system.
 *
 * @par Control-Wind Axis System (CWAS)
 * Control-Wind Axis System is very much like Control Axis System, the only
 * difference is that it is rotated about z-axis in such a manner that x-axis
 * points directly into relative wind, so there is no lateral airspeed
 * component.
 *
 * ### Refernces:
 * - Mil M.: Vertolety Raschet i proyektirovanye. Tom 1 Aerodinamika, 1966 [in Russian]
 * - Helicopters: Calculation and Design. Volume 1: Aerodynamics, NASA-TT-F-494
 * - Gessow A. Myers G.: Aerodynamics of the Helicopter, 1985
 * - Bramwell A.: Bramwells Helicopter Dynamics, 2001
 * - Padfield G.: Helicopter Flight Dynamics, 2007
 * - [Rotary-Wing Aerodynamics. Volume I, Basic Theories of Rotor Aerodynamics, NASA-CR-3082](https://ntrs.nasa.gov/citations/19790013868)
 * - Johnson W.: Helicopter Theory, 1980
 * - [Model for Vortex Ring State Influence on Rotorcraft Flight Dynamics, NASA-TP-2005-213477](https://ntrs.nasa.gov/citations/20060024029)
 */
class MCSIMAPI MainRotor
{
public:

    /**
     * @brief Main rotor data.
     */
    struct Data
    {
        Vector3 r_hub_bas;              ///< [m]   rotor hub coordinates expressed in BAS
        Angles  a_hub_bas;              ///< [rad] rotor hub orientation relative to BAS

        bool ccw = false;               ///< specifies if rotor rotation direction is counter-clockwise

        int nb = 0;                     ///< number of rotor blades

        double blade_mass = 0.0;        ///< [kg] single blade mass

        double r = 0.0;                 ///< [m] rotor radius
        double c = 0.0;                 ///< [m] blades chord
        double e = 0.0;                 ///< [m] flapping hinge offset

        double a = 0.0;                 ///< [1/rad] blade section lift curve slope
        double b = 0.0;                 ///< [-] tip losses coefficient

        double delta_0 = 0.0;           ///< [-] drag coefficient constant component
        double delta_2 = 0.0;           ///< [-] drag coefficient quadratic component

        double beta_max = 0.0;          ///< [rad] maximum flapping angle

        double ct_max = DBL_MAX;        ///< [-] maximum thrust coefficient
        double ch_max = DBL_MAX;        ///< [-] maximum hforce coefficient
        double cq_max = DBL_MAX;        ///< [-] maximum torque coefficient

        double thrust_factor = 1.0;     ///< [-] thrust tuning factor
        double hforce_factor = 1.0;     ///< [-] hforce tuning factor
        double torque_factor = 1.0;     ///< [-] torque tuning factor

        double c_ige_max = DBL_MAX;     ///< [-] maximum In-Ground-Effect influence coefficient

        double ige_thrust_factor = 1.0; ///< [-] In-Ground-Effect influence coefficient tuning factor for thrust
        double ige_torque_factor = 1.0; ///< [-] In-Ground-Effect influence coefficient tuning factor for torque

        double vrs_thrust_factor = 1.0; ///< [-] Vortex-Ring-State influence coefficient tuning factor for thrust
        double vrs_torque_factor = 1.0; ///< [-] Vortex-Ring-State influence coefficient tuning factor for torque
    };

    virtual const Data* GetData() const = 0;

    /**
     * @brief Updates force and moment.
     * @param vel_bas     [m/s]     aircraft linear velocity vector expressed in BAS
     * @param omg_bas     [rad/s]   aircraft angular velocity expressed in BAS
     * @param acc_bas     [m/s^2]   aircraft linear acceleration vector expressed in BAS
     * @param eps_bas     [rad/s^2] aircraft angular acceleration vector expressed in BAS
     * @param vel_air_bas [m/s]     aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s]   aircraft angular velocity relative to the air expressed in BAS
     * @param grav_bas    [m/s^2]   gravity acceleration vector expressed in BAS
     * @param rho         [kg/m^3]  air density
     * @param h_agl       [m]       altitude above ground level
     */
    virtual void UpdateForceAndMoment(const Vector3 &vel_bas,
                                      const Vector3 &omg_bas,
                                      const Vector3 &acc_bas,
                                      const Vector3 &eps_bas,
                                      const Vector3 &vel_air_bas,
                                      const Vector3 &omg_air_bas,
                                      const Vector3 &grav_bas,
                                      double rho,
                                      double h_agl);

    /**
     * @brief Updates main rotor model.
     * @param omega      [rad/s] rotor revolution speed
     * @param collective [rad]   collective pitch angle
     * @param cyclicLat  [rad]   cyclic lateral pitch angle
     * @param cyclicLon  [rad]   cyclic longitudinal pitch angle
     */
    virtual void Update(double omega,
                        double collective,
                        double cyclicLat,
                        double cyclicLon);

    inline const Vector3& f_bas() const { return _f_bas; }
    inline const Vector3& m_bas() const { return _m_bas; }

    inline double ir() const { return _ir; }

    inline double beta_0()  const { return _beta_0;  }
    inline double beta_1c() const { return _beta_1c; }
    inline double beta_1s() const { return _beta_1s; }

    inline double theta_0()  const { return _theta_0;  }
    inline double theta_1c() const { return _theta_1c; }
    inline double theta_1s() const { return _theta_1s; }

    inline double coning_angle() const { return _coning_angle; }
    inline double disk_roll()    const { return _disk_roll;    }
    inline double disk_pitch()   const { return _disk_pitch;   }

    inline double ct() const { return _ct; }
    inline double ch() const { return _ch; }
    inline double cq() const { return _cq; }

    inline double thrust() const { return _thrust; }
    inline double hforce() const { return _hforce; }
    inline double torque() const { return _torque; }

    inline double lambda()    const { return _lambda;    }
    inline double lambda_i()  const { return _lambda_i;  }
    inline double lambda_i0() const { return _lambda_i0; }

    inline double vel_i()  const { return _vel_i;  }
    inline double vel_i0() const { return _vel_i0; }

    inline double wake_skew() const { return _wake_skew; }

    inline bool in_vrs() const { return _in_vrs; }

protected:

    Vector3 _f_bas;                 ///< [N] total force vector expressed in BAS
    Vector3 _m_bas;                 ///< [N*m] total moment vector expressed in BAS

    Matrix3x3 _bas2ras;             ///< matrix of rotation from BAS to RAS
    Matrix3x3 _ras2bas;             ///< matrix of rotation from RAS to BAS

    Matrix3x3 _ras2cas;             ///< matrix of rotation from RAS to CAS
    Matrix3x3 _cas2ras;             ///< matrix of rotation from CAS to RAS

    Matrix3x3 _bas2cas;             ///< matrix of rotation from BAS to CAS
    Matrix3x3 _cas2bas;             ///< matrix of rotation from CAS to BAS

    Matrix3x3 _bas2das;             ///< matrix of rotation from BAS to DAS
    Matrix3x3 _das2bas;             ///< matrix of rotation from DAS to BAS

    Matrix3x3 _ras2rwas;            ///< matrix of rotation from RAS to RWAS
    Matrix3x3 _rwas2ras;            ///< matrix of rotation from RWAS to RAS

    Matrix3x3 _cas2cwas;            ///< matrix of rotation from CAS to CWAS
    Matrix3x3 _cwas2cas;            ///< matrix of rotation from CWAS to CAS

    Matrix3x3 _bas2cwas;            ///< matrix of rotation from BAS to CWAS
    Matrix3x3 _cwas2bas;            ///< matrix of rotation from CWAS to BAS

    Matrix3x3 _bas2rwas;            ///< matrix of rotation from BAS to RWAS
    Matrix3x3 _rwas2bas;            ///< matrix of rotation from RWAS to BAS

    double _r2 = 0.0;               ///< [m^2] rotor radius squared
    double _r3 = 0.0;               ///< [m^3] rotor radius cubed
    double _r4 = 0.0;               ///< [m^3] rotor radius to the power of 4
    double _b2 = 0.0;               ///< [-]   tip losses coefficient squared
    double _b3 = 0.0;               ///< [-]   tip losses coefficient cubed
    double _b4 = 0.0;               ///< [-]   tip losses coefficient to the power of 4
    double _ar = 0.0;               ///< [m^2] rotor disk area
    double _s  = 0.0;               ///< [-]   rotor solidity

    double _sb = 0.0;               ///< [kg*m]   single rotor blade first moment of mass about flapping hinge
    double _ib = 0.0;               ///< [kg*m^2] single rotor blade inertia about flapping hinge
    double _ir = 0.0;               ///< [kg*m^2] rotor total inertia about shaft axis

    double _cdir = 0.0;             ///< [-] direction coefficient (equels 1 for CCW and -1 for CW)

    double _omega  = 0.0;           ///< [rad/s]     rotor revolution speed
    double _omega2 = 0.0;           ///< [rad^2/s^2] rotor revolution speed squared
    double _omegaR = 0.0;           ///< [m/s]       rotor tip velocity

    double _beta_0       = 0.0;     ///< [rad] rotor coning angle
    double _beta_1c      = 0.0;     ///< [rad] longitudinal flapping angle
    double _beta_1s      = 0.0;     ///< [rad] lateral flapping angle
    double _beta_1c_cwas = 0.0;     ///< [rad] longitudinal flapping angle expressed in CWAS
    double _beta_1s_cwas = 0.0;     ///< [rad] lateral flapping angle expressed in CWAS

    double _theta_0  = 0.0;         ///< [rad] collective feathering angle
    double _theta_1c = 0.0;         ///< [rad] cyclic longitudinal feathering angle
    double _theta_1s = 0.0;         ///< [rad] cyclic lateral feathering angle

    double _coning_angle = 0.0;     ///< [rad] rotor coning angle
    double _disk_roll    = 0.0;     ///< [rad] rotor disk roll angle
    double _disk_pitch   = 0.0;     ///< [rad] rotor disk pitch angle

    double _ct = 0.0;               ///< [-] thrust coefficient
    double _ch = 0.0;               ///< [-] hforce coefficient
    double _cq = 0.0;               ///< [-] torque coefficient

    double _thrust = 0.0;           ///< [N]   rotor thrust
    double _hforce = 0.0;           ///< [N]   rotor hforce
    double _torque = 0.0;           ///< [N*m] rotor torque

    double _lambda    = 0.0;        ///< [-] normalized velocity at rotor disc
    double _lambda_i  = 0.0;        ///< [-] normalized rotor induced velocity
    double _lambda_i0 = 0.0;        ///< [-] normalized rotor induced velocity in hover

    double _vel_i  = 0.0;           ///< [m/s] rotor induced velocity
    double _vel_i0 = 0.0;           ///< [m/s] rotor induced velocity in hover

    double _wake_skew = 0.0;        ///< [rad] rotor wake skew angle

    bool _in_vrs = false;           ///< specifies if rotor is in a vortex ring state

    unsigned int _iter_max = 100;   ///< maximum number of iteration loop steps

    /**
     * @brief Updates auxiliary variables that are directly derived from data.
     * Examples of such variables: radius squared, disk area, solidity, etc.
     */
    virtual void UpdateDataDerivedVariables();

    /**
     * @brief Updates flapping angles and thrust coefficient.
     * @param mu_x  [-]     normalized velocity at rotor hub x-component
     * @param mu_x2 [-]     normalized velocity at rotor hub x-component squared
     * @param mu_z  [-]     normalized velocity at rotor hub z-component
     * @param p     [rad/s] rotor hub roll rate expressed in CWAS
     * @param q     [rad/s] rotor hub pitch rate expressed in CWAS
     * @param a_z   [m/s^2] rotor hub acceleration z-compoent expressed in CWAS
     * @param gamma [-]     Lock number
     */
    virtual void UpdateFlappingAnglesAndThrustCoef(double mu_x, double mu_x2, double mu_z,
                                                   double p, double q, double a_z,
                                                   double gamma);

    /**
     * @brief Updates flapping angles, thrust coefficient and induced velocity.
     * @param mu_x  [-]     normalized velocity at rotor hub x-component
     * @param mu_x2 [-]     normalized velocity at rotor hub x-component squared
     * @param mu_z  [-]     normalized velocity at rotor hub z-component
     * @param p     [rad/s] rotor hub roll rate expressed in CWAS
     * @param q     [rad/s] rotor hub pitch rate expressed in CWAS
     * @param a_z   [m/s^2] rotor hub acceleration z-compoent expressed in CWAS
     * @param gamma [-]     Lock number
     */
    virtual void UpdateFlappingAnglesThrustCoefsAndVelocity(double mu_x, double mu_x2, double mu_z,
                                                            double p, double q, double a_z,
                                                            double gamma);

    /**
     * @brief Gets In-Ground-Effect influence coefficient.
     * @param h_agl [m]   altitude above ground level
     * @param v     [m/s] rotor hub airspeed
     * @param vi    [m/s] rotor induced velocity
     */
    virtual double GetInGroundEffectThrustCoef(double h_agl, double v, double vi);

    /**
     * @brief Gets Vortex-Ring-State influence coefficient.
     * @param vx_norm [-] velcoity tangent to the rotor disc normalized by induced velocity in hover
     * @param vz_norm [-] velcoity perpendiculat to the rotor disc normalized by induced velocity in hover
     * @return Vortex-Ring-State influence coefficient
     */
    virtual double GetVortexRingInfluenceCoef(double vx_norm, double vz_norm);
};

} // namespace mc

#endif // MCSIM_ROTOR_MAINROTOR_H_
