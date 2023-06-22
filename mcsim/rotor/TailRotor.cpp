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

#include <mcsim/rotor/TailRotor.h>

#include <mcsim/aero/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void TailRotor::computeForceAndMoment( const Vector3 &vel_air_bas,
                                       const Vector3 &omg_air_bas,
                                       double airDensity )
{
    double omega2 = _omega * _omega;
    double omegaR = _omega * _data.r;

    // velocity transformations
    Vector3 vel_air_ras = _bas2ras * ( vel_air_bas + ( omg_air_bas % _data.r_hub_bas ) );

    // angle of attack
    double alpha = GetAngleOfAttack( vel_air_ras );

    // induced velocity
    double lambda = 0.0;

    const double airspeed = vel_air_ras.GetLength();

    // rotor advance ratio
    const double mu   = airspeed * cos( alpha ) / omegaR;
    const double mu2  = mu * mu;
    const double mu_z = airspeed * sin( alpha ) / omegaR;

    // thrust coefficient
    double ct = 0.0;

    // rotor inflow
    double lambda_i = _vel_i / omegaR;

    if ( fabs( lambda_i ) < 10e-14 ) lambda_i = 10e-14;

    // iteration loopp
    for ( int i = 0; i < 100; ++i )
    {
        lambda = mu_z - lambda_i;

        // thrust coefficient
        ct = 0.5 * _data.a * _s  * _data.b * ( lambda * _data.b / 2.0 + _theta * ( _data.b + 1.5 * mu2 ) / 3.0 );
        if ( ct > _data.ct_max ) ct = _data.ct_max;

        // zero function (Padfield p.124)
        double lambda_d = mu2 + lambda * lambda;
        double g_0 = lambda_i - ct / ( 2.0 * sqrt( lambda_d ) );

        // break condition
        if ( fabs( g_0 ) < 1.0e-6 ) break;

        // (Padfield p.124)
        double h_j = -( 2.0 * lambda_i * sqrt( lambda_d ) - ct ) * lambda_d
                / ( 2*pow( lambda_d, 2.0/3.0 ) + _data.a * _s * lambda_d / 4.0 - ct * lambda );

        // (Padfield p.124)
        double f_j = 1.0;
        if ( fabs( mu_z ) < 0.05 ) f_j = 0.6;

        // (Padfield p.124)
        double lambda_i_new = lambda_i + f_j * h_j;

        if ( IsValid( lambda_i_new ) ) lambda_i = lambda_i_new;
    }

    // drag coefficient
    double cd = _data.delta_0 + _data.delta_2 * ct*ct;

    // moment of resistance coefficient (Bramwell p.102)
    double cq = cd * _s * ( 1.0 + 3.0 * mu2 ) / 8.0 - lambda * ct;
    if ( cq > _data.cq_max ) cq = _data.cq_max;

    // induced velocity
    _vel_i = lambda_i * omegaR;

    _thrust = _data.thrust_factor * airDensity * _ar * _r2 * omega2 * ct;
    _torque = _data.torque_factor * airDensity * _ar * _r3 * omega2 * cq;

    _for_bas = _ras2bas * Vector3( 0.0, 0.0, -_thrust );
    _mom_bas = _data.r_hub_bas % _for_bas;

    if ( !_for_bas.IsValid() || !_mom_bas.IsValid() )
    {
        // TODO
    }
}

////////////////////////////////////////////////////////////////////////////////

void TailRotor::update( double omega, double theta )
{
    _omega = omega;
    _theta = theta;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
