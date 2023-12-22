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

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void Propeller::ComputeThrust(double airspeed, double rho)
{
    if ( rps_ > 0.0 )
    {
        double j = airspeed / ( data().diameter * rps_ );
        double c_t = data().c_t.GetValue(j, pitch_);

        thrust_ = c_t * rho * Math::Pow2(rps_) * Math::Pow4(data().diameter);
    }
    else
    {
        thrust_ = 0.0;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Propeller::Integrate(double dt, double i_eng)
{
    // integrating propeller omega
    omega_ += ( (trq_a_ - trq_r_) / (data().inertia + i_eng) ) * dt;

    rps_ = std::max(0.0, omega_ / (2.0 * M_PI));

    // engine friction stops propeller
    if ( trq_a_ < trq_r_ && rps_ < 1.0 )
    {
        rps_ = rps_ < 0.1 ? 0.0 : Inertia(0.0, rps_, dt, 0.1);
        omega_ = 2.0 * M_PI * rps_;
    }

    rpm_ = 60.0 * rps_;
}

////////////////////////////////////////////////////////////////////////////////

void Propeller::Update(double prop_lever,
                       double torque,
                       double airspeed,
                       double rho)
{
    pitch_ = GetPropellerPitch(prop_lever);

    double j = airspeed / ( data().diameter * ( rps_ > 0.1 ? rps_ : 0.1 ) );
    double c_p = data().c_p.GetValue(j, pitch_);
    double p_r = c_p * rho * Math::Pow3(rps_) * Math::Pow5(data().diameter);

    vel_i_ = GetInducedVelocity(airspeed, rho);

    trq_r_ = p_r / ( omega_ > 1.0 ? omega_ : 1.0 );
    trq_a_ = torque / data().gear_ratio;
    trq_n_ = std::min(trq_r_, trq_a_);
}

////////////////////////////////////////////////////////////////////////////////

void Propeller::set_rpm(double rpm)
{
    rpm_ = std::max(0.0, rpm);
    rps_ = rpm_ / 60.0;
    omega_ = 2.0 * M_PI * rps_;
}

////////////////////////////////////////////////////////////////////////////////

double Propeller::GetInducedVelocity(double airspeed, double rho)
{
    double vi = 0.0;

    // 0.5*rho*A*vi^2 + rho*A*V*vi - T = 0
    // a = 0.5*rho*A
    // b = rho*A*V
    // c = -T
    double a = 0.5 * rho * area_;
    double b = rho * area_ * airspeed;
    double c = -thrust_;

    double delta = b*b - 4.0*a*c;
    if ( delta >= 0.0 )
    {
        // the 2nd result has no physical meaning
        vi = ( -b + sqrt(delta) ) / ( 2.0 * a );
    }

    return vi;
}

////////////////////////////////////////////////////////////////////////////////

double Propeller::GetPropellerPitch(double prop_lever)
{
    return data().prop_pitch.GetValue(prop_lever);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
