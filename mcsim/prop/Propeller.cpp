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

#include <mcsim/prop/Propeller.h>

#include <algorithm>
#include <cmath>

#include <mcutils/math/Math.h>
#include <mcutils/misc/String.h>
#include <mcutils/misc/Units.h>
#include <mcutils/physics/Physics.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void Propeller::computeThrust( double airspeed, double airDensity )
{
    if ( _speed_rps > 0.0 )
    {
        double advance = airspeed / ( _data.diameter * _speed_rps );
        double coefThrust = _data.coefThrust.GetValue( advance, _pitch );

        _thrust = coefThrust * airDensity
                * Math::Pow2( _speed_rps )
                * Math::Pow4( _data.diameter );
    }
    else
    {
        _thrust = 0.0;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Propeller::integrate( double dt, double engineInertia )
{
    // integrating propeller omega
    _omega += ( ( _torqueAvailable - _torqueRequired ) / ( _data.inertia + engineInertia ) ) * dt;

    _speed_rps = std::max( 0.0, _omega / ( 2.0 * M_PI ) );

    // engine friction stops propeller
    if ( _torqueAvailable < _torqueRequired && _speed_rps < 1.0 )
    {
        _speed_rps = _speed_rps < 0.1 ? 0.0 : Physics::inertia( 0.0, _speed_rps, dt, 0.1 );
        _omega = 2.0 * M_PI * _speed_rps;
    }

    _speed_rpm = 60.0 * _speed_rps;
}

////////////////////////////////////////////////////////////////////////////////

void Propeller::update( double propellerLever,
                        double engineTorque,
                        double airspeed,
                        double airDensity )
{
    _pitch = getPropellerPitch( propellerLever );

    double advance = airspeed / ( _data.diameter * ( _speed_rps > 0.1 ? _speed_rps : 0.1 ) );
    double coefPower = _data.coefPower.GetValue( advance, _pitch );
    double powerRequired = coefPower * airDensity
            * Math::Pow3( _speed_rps )
            * Math::Pow5( _data.diameter );

    _inducedVelocity = getInducedVelocity( airspeed, airDensity );

    _torqueRequired  = powerRequired / ( _omega > 1.0 ? _omega : 1.0 );
    _torqueAvailable = engineTorque / _data.gearRatio;
}

////////////////////////////////////////////////////////////////////////////////

void Propeller::setRPM( double rpm )
{
    _speed_rpm = std::max( 0.0, rpm );
    _speed_rps = _speed_rpm / 60.0;
    _omega = 2.0 * M_PI * _speed_rps;
}

////////////////////////////////////////////////////////////////////////////////

double Propeller::getInducedVelocity( double airspeed, double airDensity )
{
    double vi = 0.0;

    // 0.5*rho*A*vi^2 + rho*A*V*vi - T = 0
    // a = 0.5*rho*A
    // b = rho*A*V
    // c = -T
    double a = 0.5 * airDensity * _area;
    double b = airDensity * _area * airspeed;
    double c = -_thrust;

    double delta = b*b - 4.0*a*c;

    if ( delta >= 0.0 )
    {
        // the 2nd result has no physical meaning
        vi = ( -b + sqrt( delta ) ) / ( 2.0 * a );
    }

    return vi;
}

////////////////////////////////////////////////////////////////////////////////

double Propeller::getPropellerPitch( double propellerLever )
{
    return _data.propPitch.GetValue( propellerLever );
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
