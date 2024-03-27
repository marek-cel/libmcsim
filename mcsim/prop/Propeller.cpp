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
#include <mcutils/misc/Units.h>
#include <mcutils/physics/Inertia.h>

namespace mc {

void Propeller::ComputeThrust(double airspeed, double rho)
{
    if ( _rps > 0.0 )
    {
        double j = airspeed / (data().diameter * _rps);
        double c_t = data().c_t.GetValue(j, _pitch);

        _thrust = c_t * rho * Math::Pow2(_rps) * Math::Pow4(data().diameter);
    }
    else
    {
        _thrust = 0.0;
    }
}

void Propeller::Integrate(double dt, double i_eng)
{
    // integrating propeller omega
    _omega += ((_trq_a - _trq_r) / (data().inertia + i_eng)) * dt;

    _rps = std::max(0.0, _omega / (2.0 * M_PI));

    // engine friction stops propeller
    if ( _trq_a < _trq_r && _rps < 1.0 )
    {
        _rps = _rps < 0.1 ? 0.0 : FirstOrderInertia(0.0, _rps, dt, 0.1);
        _omega = 2.0 * M_PI * _rps;
    }

    _rpm = 60.0 * _rps;
}

void Propeller::Update(double prop_lever,
                       double torque,
                       double airspeed,
                       double rho)
{
    _pitch = GetPropellerPitch(prop_lever);

    double j = airspeed / ( data().diameter * ( _rps > 0.1 ? _rps : 0.1 ) );
    double c_p = data().c_p.GetValue(j, _pitch);
    double p_r = c_p * rho * Math::Pow3(_rps) * Math::Pow5(data().diameter);

    _vel_i = GetInducedVelocity(airspeed, rho);

    _trq_r = p_r / ( _omega > 1.0 ? _omega : 1.0 );
    _trq_a = torque / data().gear_ratio;
    _trq_n = std::min(_trq_r, _trq_a);
}

void Propeller::set_rpm(double rpm)
{
    _rpm = std::max(0.0, rpm);
    _rps = _rpm / 60.0;
    _omega = 2.0 * M_PI * _rps;
}

double Propeller::GetInducedVelocity(double airspeed, double rho)
{
    double vi = 0.0;

    // 0.5*rho*A*vi^2 + rho*A*V*vi - T = 0
    // a = 0.5*rho*A
    // b = rho*A*V
    // c = -T
    double a = 0.5 * rho * _area;
    double b = rho * _area * airspeed;
    double c = -_thrust;

    double delta = b*b - 4.0*a*c;
    if ( delta >= 0.0 )
    {
        // the 2nd result has no physical meaning
        vi = ( -b + sqrt(delta) ) / ( 2.0 * a );
    }

    return vi;
}

double Propeller::GetPropellerPitch(double prop_lever)
{
    return data().prop_pitch.GetValue(prop_lever);
}

} // namespace mc
