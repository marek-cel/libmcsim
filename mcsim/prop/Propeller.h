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
#ifndef MCSIM_PROP_PROPELLER_H_
#define MCSIM_PROP_PROPELLER_H_

#include <memory>

#include <mcutils/math/Table.h>
#include <mcutils/math/Table2.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

namespace mc {

/**
 * @brief Propeller class.
 *
 * ### Refernces:
 * - Allerton D. Principles of Flight Simulation, 2009, p.131
 * - Raymer D. Aircraft Design: A Conceptual Approach, 1992, p.327
 * - Torenbeek E. Synthesis of Subsonic Airplane Design, 1982, p.191
 * - Paturski Z. Przewodnik po projektach z Mechaniki Lotu, Projekt nr 5: Charakterystyki zespolu napedowego. [in Polish]
 */
class MCSIMAPI Propeller
{
public:

    /**
     * @brief Propeller data struct.
     */
    struct Data
    {
        Vector3 pos_bas;            ///< [m] propeller position expressed in BAS

        Table prop_pitch;           ///< [rad] propeller pitch vs [-] normalized pitch

        Table2 c_t;                 ///< [-] thrust coefficient
        Table2 c_p;                 ///< [-] power coefficient

        double gear_ratio = 0.0;    ///< [-] gear ratio (propeller rpm / engine rpm)
        double diameter   = 0.0;    ///< [m] diameter
        double inertia    = 0.0;    ///< [kg*m^2] polar moment of inertia

        bool ccw = false;           ///< specifies if rotation direction is counter-clockwise (looking from cockpit)
    };

    /**
     * @brief Computes thrust.
     * @param airspeed [m/s]    airspeed
     * @param rho      [kg/m^3] air density
     */
    virtual void ComputeThrust(double airspeed, double rho);

    /**
     * @brief Integrates model.
     * @param dt [s] time step
     * @param i_eng [kg*m^2] engine polar moment of inertia
     */
    virtual void Integrate(double dt, double i_eng);

    /**
     * @brief Updates propeller.
     * @param prop_lever <0.0;1.0> normalized propeller lever position
     * @param torque     [N]       engine torque
     * @param airspeed   [m/s]     airspeed
     * @param rho        [kg/m^3]  air density
     */
    virtual void Update(double prop_lever,
                        double torque,
                        double airspeed,
                        double rho);

    virtual const Data& data() const = 0;

    inline double rpm()    const { return _rpm;    }
    inline double omega()  const { return _omega;  }
    inline double thrust() const { return _thrust; }
    inline double vel_i()  const { return _vel_i;  }
    inline double trq_n()  const { return _trq_n;  }

    void set_rpm(double rpm);

protected:

    double _area = 0.0;             ///< [m^2] propeller disc area

    double _pitch  = 0.0;           ///< [rad]   propeller pitch at 0.75 radius
    double _omega  = 0.0;           ///< [rad/s] propeller angular velocity
    double _rps    = 0.0;           ///< [rps]   propeller speed
    double _rpm    = 0.0;           ///< [rpm]   propeller speed
    double _thrust = 0.0;           ///< [N] t   hrust

    double _vel_i = 0.0;            ///< [m/s] induced velocity

    double _trq_a = 0.0;            ///< [N*m] available torque
    double _trq_r = 0.0;            ///< [N*m] required torque
    double _trq_n = 0.0;            ///< [N*m] net torque

    /**
     * @brief Computes induced velocity.
     * @param airspeed [m/s] airspeed
     * @param rho [kg/m^3] air density
     */
    virtual double GetInducedVelocity(double airspeed, double rho);

    /**
     * @brief Computes propeller pitch.
     * @param prop_lever <0.0;1.0> normalized propeller lever position
     */
    virtual double GetPropellerPitch(double prop_lever);
};

} // namespace mc

#endif // MCSIM_PROP_PROPELLER_H_
