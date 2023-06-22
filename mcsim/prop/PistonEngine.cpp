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

void PistonEngine::Update( double throttleLever,
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

    rpm_ = rpm;
    map_ = GetManifoldAbsolutePressure( throttleLever, rpm_, airPressure );
    pwr_ = GetNetPower( throttleLever, mixtureLever, rpm_, airDensity, densityAlt,
                          fuel, magneto_l, magneto_r );

    af_ = 0.5 * data_->displacement * airDensity * ( rpm_ / 60.0 );
    ff_ = std::max( 0.0, pwr_ ) * data_->specFuelCons;

    // engine torque [N*m]
    trq_ = ( omega > 1.0 ) ? pwr_ / omega : pwr_;

    // state
    if ( pwr_ > 0.0 || ( rpm_ > data_->rpm_min && fuel && ( magneto_l || magneto_r ) ) )
    {
        state_ = State::Running;
    }
    else
    {
        state_ = State::Stopped;

        if ( starter )
        {
            state_ = State::Starting;
            trq_ += data_->starter;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void PistonEngine::set_rpm( double rpm )
{
    rpm_ = std::max(0.0, rpm);
}

////////////////////////////////////////////////////////////////////////////////

double PistonEngine::GetManifoldAbsolutePressure( double throttleLever,
                                                  double rpm, double airPressure )
{
    double map = airPressure
            * data_->map_throttle.GetValue( throttleLever )
            * data_->map_rpm.GetValue( rpm );

    map = std::max( 0.0, map );

    return map;
}

////////////////////////////////////////////////////////////////////////////////

double PistonEngine::GetFuelToAirRatio( double mixture, double airDensity )
{
    return mixture * (1.225 / airDensity );
}

////////////////////////////////////////////////////////////////////////////////

double PistonEngine::GetPowerFactor( double mixture, double airDensity, bool fuel,
                                     bool magneto_l, bool magneto_r )
{
    double fuelToAirRatio = GetFuelToAirRatio( mixture, airDensity );

    // Allerton D.: Principles of Flight Simulation, p.130
    double powerFactor = data_->power_factor.GetValue( fuelToAirRatio );

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

double PistonEngine::GetNetPower( double throttleLever, double mixtureLever, double rpm,
                                  double airDensity, double densityAltitude,
                                  bool fuel, bool magneto_l, bool magneto_r )
{
    double power = data_->power_rpm.GetValue( rpm );
    power *= data_->power_throttle.GetValue( throttleLever );
    power *= data_->power_altitude.GetValue( densityAltitude );
    power *= GetPowerFactor( data_->mixture.GetValue( mixtureLever ), airDensity,
                             fuel, magneto_l, magneto_r );

    if ( rpm < data_->rpm_min ) power = 0.0;

    return power;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
