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
#ifndef MCSIM_PROP_PISTONENGINE_H_
#define MCSIM_PROP_PISTONENGINE_H_

#include <memory>

#include <mcutils/math/Table.h>

#include <mcsim/defs.h>

namespace mc {

/**
 * @brief Piston engine model class.
 *
 * ### Refernces:
 * - Allerton D. Principles of Flight Simulation, 2009, p.128
 */
class MCSIMAPI PistonEngine
{
public:

    /** Engine state enum. */
    enum class State
    {
        Stopped  = 0,   ///< engine stopped
        Starting = 1,   ///< engine starting
        Running  = 2    ///< engine running
    };

    /**
     * @brief Engine data struct.
     */
    struct Data
    {
        Table power_rpm;            ///< [W] power vs engine rpm
        Table power_throttle;       ///< [-] power coefficient vs throttle
        Table power_altitude;       ///< [-] power coefficient vs altitude
        Table mixture;              ///< [-] mixture vs mixture lever position
        Table power_factor;         ///< [-] power factor vs fuel to air ratio
        Table map_throttle;         ///< [-] manifold absolute pressure ratio due to throttle
        Table map_rpm;              ///< [-] manifold absolute pressure ratio due to engine rpm

        double rpm_min      = 0.0;  ///< [rpm]      engine minimum rpm
        double rpm_max      = 0.0;  ///< [rpm]      engine maximum rpm
        double starter      = 0.0;  ///< [N*m]      starter torque
        double displacement = 0.0;  ///< [m^3]      displacement
        double inertia      = 0.0;  ///< [kg*m^2]   polar moment of inertia
        double specFuelCons = 0.0;  ///< [kg/(W*s)] specific fuel consumption
    };

    /**
     * @brief Updates engine.
     * @param throttleLever [0.0,1.0] throttle lever position
     * @param mixtureLever  [0.0,1.0] mixture lever position
     * @param rpm           [rpm]     engine rpm
     * @param airPressure   [Pa]      air pressure
     * @param airDensity    [kg/m^3]  air density
     * @param densityAlt    [m]       air density altitude
     * @param fuel          specifies if fuel is provided
     * @param starter       specifies if starter is enabled
     * @param magneto_l     specifies if left magneto is enabled
     * @param magneto_r     specifies if right magneto is enabled
     */
    virtual void Update(double throttleLever,
                        double mixtureLever,
                        double rpm,
                        double airPressure,
                        double airDensity,
                        double densityAlt,
                        bool fuel,
                        bool starter,
                        bool magneto_l = true,
                        bool magneto_r = true);

    virtual const Data& data() const = 0;

    inline State state() const { return _state; }

    inline double rpm() const { return _rpm; }
    inline double map() const { return _map; }
    inline double pwr() const { return _pwr; }
    inline double trq() const { return _trq; }
    inline double af()  const { return _af;  }
    inline double ff()  const { return _ff;  }

    void set_rpm(double rpm);

protected:

    State _state = State::Stopped;  ///< engine state

    double _rpm = 0.0;              ///< [rpm]  engine rpm
    double _map = 0.0;              ///< [Pa]   manifold absolute pressure
    double _pwr = 0.0;              ///< [W]    net power
    double _trq = 0.0;              ///< [N*m]  torque
    double _af  = 0.0;              ///< [kg/s] air flow
    double _ff  = 0.0;              ///< [kg/s] fuel flow

    /**
     * @brief Computes manifold absolute pressure.
     * @param throttleLever [0.0,1.0] throttle lever position
     * @param rpm [rpm] engine rpm
     * @param press [Pa] air pressure
     * @return [Pa] manifold absolute pressure
     */
    virtual double GetManifoldAbsolutePressure(double throttleLever,
                                               double rpm, double press);

    /**
     * @brief Computes fuel to air ratio.
     * @param mixture [-] mixture
     * @param rho [kg/m^3] air density
     * @return [-] fuel to air ratio
     */
    virtual double GetFuelToAirRatio(double mixture, double rho);

    /**
     * @brief Computes engine power factor.
     * @param fuel specifies if fuel is provided
     * @param mixture [-] mixture
     * @param rho [kg/m^3] air density
     * @param magneto_l specifies if left magneto is enabled
     * @param magneto_r specifies if right magneto is enabled
     * @return [-] power factor
     */
    virtual double GetPowerFactor(double mixture, double rho, bool fuel,
                                  bool magneto_l = true, bool magneto_r = true);

    /**
     * @brief Computes engine net power.
     * @param throttleLever [0.0,1.0] throttle lever position
     * @param mixtureLever [0.0,1.0] mixture lever position
     * @param rpm [rpm] engine rpm
     * @param rho [kg/m^3] air density
     * @param densityAltitude [m] air density altitude
     * @param fuel specifies if fuel is provided
     * @param magneto_l specifies if left magneto is enabled
     * @param magneto_r specifies if right magneto is enabled
     * @return [W] net power
     */
    virtual double GetNetPower(double throttleLever, double mixtureLever, double rpm,
                               double rho, double densityAltitude,
                               bool fuel, bool magneto_l, bool magneto_r);
};

} // namespace mc

#endif // MCSIM_PROP_PISTONENGINE_H_
