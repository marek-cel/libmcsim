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

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Piston engine model class.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Allerton D. Principles of Flight Simulation, 2009, p.128</li>
 * </ul>
 */
class MCSIMAPI PistonEngine
{
public:

    /** Engine state enum. */
    enum State
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

    // LCOV_EXCL_START
    PistonEngine() = default;
    PistonEngine(const PistonEngine&) = delete;
    PistonEngine(PistonEngine&&) = default;
    PistonEngine& operator=(const PistonEngine&) = delete;
    PistonEngine& operator=(PistonEngine&&) = default;
    virtual ~PistonEngine() = default;
    // LCOV_EXCL_STOP

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
    virtual void update( double throttleLever,
                         double mixtureLever,
                         double rpm,
                         double airPressure,
                         double airDensity,
                         double densityAlt,
                         bool fuel,
                         bool starter,
                         bool magneto_l = true,
                         bool magneto_r = true );

    /**
     * @brief Returns engine state.
     * @return engine state
     */
    inline State getState() const
    {
        return _state;
    }

    /**
     * @brief Returns engine air flow.
     * @return [kg/s] engine air flow
     */
    inline double getAirFlow() const
    {
        return _airFlow;
    }

    /**
     * @brief Returns engine fuel consumption.
     * @return [kg/s] engine fuel consumption
     */
    inline double getFuelFlow() const
    {
        return _fuelFlow;
    }

    /**
     * @brief Returns engine polar moment of inertia.
     * @return [kg*m^2] engine polar moment of inertia
     */
    inline double getInertia() const
    {
        return _data.inertia;
    }

    /**
     * @brief Returns engine manifold absolute pressure.
     * @return [Pa] engine manifold absolute pressure
     */
    inline double getMAP() const
    {
        return _map;
    }

    /**
     * @brief Returns engine net power.
     * @return [W] engine net power
     */
    inline double getPower()  const
    {
        return _power;
    }

    /**
     * @brief Returns engine rpm.
     * @return [rpm] engine rpm
     */
    inline double getRPM() const
    {
        return _rpm;
    }

    /**
     * @brief Returns engine torque.
     * @return [N*m] engine torque
     */
    inline double getTorque() const
    {
        return _torque;
    }

    void setRPM( double rpm );

protected:

    Data _data;                 ///<

    State _state { Stopped };   ///< engine state

    double _rpm      { 0.0 };   ///< [rpm]  engine rpm
    double _map      { 0.0 };   ///< [Pa]   manifold absolute pressure
    double _power    { 0.0 };   ///< [W]    net power
    double _torque   { 0.0 };   ///< [N*m]  torque
    double _airFlow  { 0.0 };   ///< [kg/s] air flow
    double _fuelFlow { 0.0 };   ///< [kg/s] fuel flow

    /**
     * @brief Computes manifold absolute pressure.
     * @param throttleLever [0.0,1.0] throttle lever position
     * @param rpm [rpm] engine rpm
     * @param airPressure [Pa] air pressure
     * @return [Pa] manifold absolute pressure
     */
    virtual double getManifoldAbsolutePressure( double throttleLever,
                                                double rpm, double airPressure );

    /**
     * @brief Computes fuel to air ratio.
     * @param mixture [-] mixture
     * @param airDensity [kg/m^3] air density
     * @return [-] fuel to air ratio
     */
    virtual double getFuelToAirRatio( double mixture, double airDensity );

    /**
     * @brief Computes engine power factor.
     * @param fuel specifies if fuel is provided
     * @param mixture [-] mixture
     * @param airDensity [kg/m^3] air density
     * @param magneto_l specifies if left magneto is enabled
     * @param magneto_r specifies if right magneto is enabled
     * @return [-] power factor
     */
    virtual double getPowerFactor( double mixture, double airDensity, bool fuel,
                                   bool magneto_l = true, bool magneto_r = true );

    /**
     * @brief Computes engine net power.
     * @param throttleLever [0.0,1.0] throttle lever position
     * @param mixtureLever [0.0,1.0] mixture lever position
     * @param rpm [rpm] engine rpm
     * @param airDensity [kg/m^3] air density
     * @param densityAltitude [m] air density altitude
     * @param fuel specifies if fuel is provided
     * @param magneto_l specifies if left magneto is enabled
     * @param magneto_r specifies if right magneto is enabled
     * @return [W] net power
     */
    virtual double getNetPower( double throttleLever, double mixtureLever, double rpm,
                                double airDensity, double densityAltitude,
                                bool fuel, bool magneto_l, bool magneto_r );
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_PROP_PISTONENGINE_H_
