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

#include <mcutils/math/Math.h>

#include <mcsim/aero/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void TailRotor::UpdateForceAndMoment(const Vector3& vel_air_bas,
                                     const Vector3& omg_air_bas,
                                     double airDensity)
{
    // velocity transformations
    Vector3 vel_air_ras = bas2ras_ * (vel_air_bas + (omg_air_bas % data_.r_hub_bas));

    // angle of attack
    double alpha = GetAngleOfAttack(vel_air_ras);

    const double airspeed = vel_air_ras.GetLength();

    // rotor advance ratio
    const double mu   = airspeed * cos(alpha) / omegaR_;
    const double mu2  = mu * mu;
    const double mu_z = airspeed * sin(alpha) / omegaR_;

    // thrust coefficient
    double ct = 0.0;

    // rotor inflow
    lambda_i_ = vel_i_ / omegaR_;

    if ( fabs(lambda_i_) < 10e-14 ) lambda_i_ = 10e-14;

    // iteration loopp
    for ( unsigned int i = 0; i < n_max_; ++i )
    {
        lambda_ = mu_z - lambda_i_;

        // thrust coefficient
        ct = 0.5 * data_.a * s_  * data_.b * (lambda_ * data_.b / 2.0 + theta_ * (data_.b + 1.5 * mu2) / 3.0);
        ct = Math::Satur(-data().ct_max, data().ct_max, ct);

        double lambda_i0_new = sqrt(0.5 * ct);
        if ( IsValid(lambda_i0_new) ) lambda_i0_ = lambda_i0_new;

        // zero function (Padfield p.124)
        double lambda_d = mu2 + Math::Pow2(lambda_);
        double g_0 = lambda_i_ - ct / (2.0 * sqrt(lambda_d));

        // break condition
        if ( i > 0 && fabs(g_0) < 1.0e-6 ) break;

        // (Padfield p.124)
        double h_j = -( 2.0 * lambda_i_ * sqrt(lambda_d) - ct ) * lambda_d
                / (2.0*pow(lambda_d, 2.0/3.0) + data_.a*s_*lambda_d / 4.0 - ct*lambda_);

        // (Padfield p.124)
        double f_j = 1.0;
        if ( fabs(mu_z) < 0.05 ) f_j = 0.6;

        // (Padfield p.124)
        double lambda_i_new = lambda_i_ + f_j * h_j;
        if ( IsValid(lambda_i_new) ) lambda_i_ = lambda_i_new;
    }

    // drag coefficient
    double cd = data_.delta_0 + data_.delta_2 * Math::Pow2(ct);

    // moment of resistance coefficient (Bramwell p.102)
    double cq = cd * s_ * ( 1.0 + 3.0 * mu2 ) / 8.0 - lambda_ * ct;
    if ( cq > data_.cq_max ) cq = data_.cq_max;

    // induced velocity (Padfield p.117)
    vel_i_  = lambda_i_  * omegaR_;
    vel_i0_ = lambda_i0_ * omegaR_;

    thrust_ = data_.thrust_factor * airDensity * ar_ * r2_ * omega2_ * ct;
    torque_ = data_.torque_factor * airDensity * ar_ * r3_ * omega2_ * cq;

    f_bas_ = ras2bas_ * Vector3(0.0, 0.0, -thrust_);
    m_bas_ = data_.r_hub_bas % f_bas_;
}

////////////////////////////////////////////////////////////////////////////////

void TailRotor::Update(double omega, double collective)
{
    omega_  = omega;
    omega2_ = omega * omega;
    omegaR_ = omega * data().r;

    theta_ = collective;
}

////////////////////////////////////////////////////////////////////////////////

void TailRotor::UpdateDataDerivedVariables()
{
    bas2ras_ = Matrix3x3(data().a_hub_bas);
    ras2bas_ = bas2ras_.GetTransposed();

    r2_ = data().r * data().r;
    r3_ = data().r * r2_;

    ar_ = M_PI * r2_;
    s_ = (static_cast<double>(data().nb)) * data().c * data().r / ar_;

    ib_ = data().blade_mass * r2_ / 3.0;
    ir_ = data().nb * ib_;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
