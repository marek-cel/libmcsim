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

#include <mcsim/prop/PistonEngine.h>

#include <algorithm>

#include <mcutils/math/Math.h>
#include <mcutils/misc/Units.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void PistonEngine::update( double throttleLever,
                           double mixtureLever,
                           double rpm,
                           double airPressure,
                           double airDensity,
                           double densityAlt,
                           bool fuel,
                           bool starter,
                           bool magneto_l,
                           bool magneto_r )
{
    double omega = M_PI * rpm / 30.0;

    _rpm = rpm;
    _map = getManifoldAbsolutePressure( throttleLever, _rpm, airPressure );
    _power = getNetPower( throttleLever, mixtureLever, _rpm, airDensity, densityAlt,
                          fuel, magneto_l, magneto_r );

    _airFlow = 0.5 * _data.displacement * airDensity * ( _rpm / 60.0 );
    _fuelFlow = std::max( 0.0, _power ) * _data.specFuelCons;

    // engine torque [N*m]
    _torque = ( omega > 1.0 ) ? _power / omega : _power;

    // state
    if ( _power > 0.0 || ( _rpm > _data.rpm_min && fuel && ( magneto_l || magneto_r ) ) )
    {
        _state = Running;
    }
    else
    {
        _state = Stopped;

        if ( starter )
        {
            _state = Starting;
            _torque += _data.starter;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void PistonEngine::setRPM( double rpm )
{
    _rpm = std::max( 0.0, rpm );
}

////////////////////////////////////////////////////////////////////////////////

double PistonEngine::getManifoldAbsolutePressure( double throttleLever,
                                                  double rpm, double airPressure )
{
    double map = airPressure
            * _data.map_throttle.GetValue( throttleLever )
            * _data.map_rpm.GetValue( rpm );

    map = std::max( 0.0, map );

    return map;
}

////////////////////////////////////////////////////////////////////////////////

double PistonEngine::getFuelToAirRatio( double mixture, double airDensity )
{
    return mixture * (1.225 / airDensity );
}

////////////////////////////////////////////////////////////////////////////////

double PistonEngine::getPowerFactor( double mixture, double airDensity, bool fuel,
                                     bool magneto_l, bool magneto_r )
{
    double fuelToAirRatio = getFuelToAirRatio( mixture, airDensity );

    // Allerton D.: Principles of Flight Simulation, p.130
    double powerFactor = _data.power_factor.GetValue( fuelToAirRatio );

    if ( !fuel )
    {
        // no fuel - engine starving
        powerFactor = 0.0;
    }
    else if ( !magneto_l && !magneto_r )
    {
        // both magnetos disabled
        powerFactor = 0.0;
    }
    else if ( ( magneto_l && !magneto_r ) || ( !magneto_l && magneto_r ) )
    {
        // 5% reduction in power caused by the reduced effectiveness of the combustion
        // Allerton D.: Principles of Flight Simulation, p.131
        powerFactor *= 0.95;
    }

    powerFactor = Math::Satur( 0.0, 1.0, powerFactor );

    return powerFactor;
}

////////////////////////////////////////////////////////////////////////////////

double PistonEngine::getNetPower( double throttleLever, double mixtureLever, double rpm,
                                  double airDensity, double densityAltitude,
                                  bool fuel, bool magneto_l, bool magneto_r )
{
    double power = _data.power_rpm.GetValue( rpm );
    power *= _data.power_throttle.GetValue( throttleLever );
    power *= _data.power_altitude.GetValue( densityAltitude );
    power *= getPowerFactor( _data.mixture.GetValue( mixtureLever ), airDensity,
                             fuel, magneto_l, magneto_r );

    if ( rpm < _data.rpm_min ) power = 0.0;

    return power;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
