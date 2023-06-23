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

#include <mcsim/rotor/MainRotor.h>

#include <mcutils/math/Math.h>
#include <mcutils/misc/String.h>
#include <mcutils/misc/Units.h>

#include <mcsim/aero/AeroAngles.h>
#include <mcsim/rotor/RotorUtils.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

MainRotor::MainRotor( std::shared_ptr<IInGroundEffect>  ige,
                      std::shared_ptr<IVortexRingState> vrs )
    : ige_(ige)
    , vrs_(vrs)
{}

////////////////////////////////////////////////////////////////////////////////

void MainRotor::ComputeForceAndMoment(const Vector3 & /*vel_bas*/,
                                      const Vector3 &omg_bas,
                                      const Vector3 &acc_bas,
                                      const Vector3 &eps_bas,
                                      const Vector3 &vel_air_bas,
                                      const Vector3 &omg_air_bas,
                                      const Vector3 &grav_bas,
                                      double rho)
{
    // Lock number
    double gamma = rho * data().a * data().c * r4_ / ib_;

    // RAS <-> CAS
    ras2cas_ = Matrix3x3(Angles(theta_1c_, theta_1s_, 0.0));

    cas2ras_ = ras2cas_.GetTransposed();

    // BAS -> CAS
    bas2cas_ = bas2ras_ * ras2cas_;

    // velocity transformations
    Vector3 vel_air_ras = bas2ras_ * ( vel_air_bas + ( omg_air_bas % data().r_hub_bas ) );
    Vector3 vel_air_cas = ras2cas_ * vel_air_ras;

    // sideslip angle
    double beta_cas = GetSideslipAngle(vel_air_cas);
    double beta_ras = GetSideslipAngle(vel_air_ras);

    // RAS -> RWAS
    ras2rwas_ = Matrix3x3(Angles(0.0, 0.0, beta_ras));
    rwas2ras_ = ras2rwas_.GetTransposed();

    // CAS <-> CWAS
    cas2cwas_ = Matrix3x3( Angles( 0.0, 0.0, beta_cas ) );
    cwas2cas_ = cas2cwas_.GetTransposed();

    // BAS <-> CWAS
    bas2cwas_ = bas2cas_ * cas2cwas_;
    cwas2bas_ = bas2cwas_.GetTransposed();

    // BAS <-> RWAS
    bas2rwas_ = bas2ras_ * ras2rwas_;
    rwas2bas_ = bas2rwas_.GetTransposed();

    // velocity transformations
    Vector3 vel_air_rwas = ras2rwas_ * vel_air_ras;
    Vector3 vel_air_cwas = cas2cwas_ * vel_air_cas;
    Vector3 omg_air_cwas = bas2cwas_ * omg_air_bas;

    // acceleration
    Vector3 acc_hub_bas = acc_bas
                        + ( omg_bas % ( omg_bas % data().r_hub_bas ) )
                        + ( eps_bas % data().r_hub_bas )
                        - grav_bas;
    Vector3 acc_hub_cwas = bas2cwas_ * acc_hub_bas;

    // angle of attack
    double alpha = GetAngleOfAttack(vel_air_rwas);

    // flapping coefficients
    double beta_1c_cwas = 0.0;
    double beta_1s_cwas = 0.0;

    const double p = omg_air_cwas.p();
    const double q = omg_air_cwas.q();

    const double airspeed = vel_air_cwas.GetLength();

    // rotor advance ratio
    const double mu_x  = airspeed * cos(alpha) / omegaR_;
    const double mu_x2 = mu_x * mu_x;
    const double mu_z  = airspeed * sin(alpha) / omegaR_;

    ct_ = 0.0;
    ch_ = 0.0;
    cq_ = 0.0;

    const double a_z = -acc_hub_cwas.z();

    // hover and climb
    UpdateFlappingAnglesThrustCoefsAndVelocity(mu_x, mu_x2, mu_z, p, q, a_z, gamma);

    double mu_z_norm =  mu_z / lambda_i0_;
    double mu_c_norm = -mu_z_norm;
    double mu_x_norm =  mu_x / lambda_i0_;

    // vortex ring, turbulent wake and windmill
    if ( mu_c_norm < -1.0 )
    {
        double ct_max = ct_;

        if ( mu_c_norm < -2.0 )
        {
            // windmill
            // Padfield: Helicopter Flight Dynamics, p.118
            double mu_z_half = 0.5 * mu_z;
            lambda_i_ = mu_z_half - sqrt(mu_z_half*mu_z_half - Math::Pow2(lambda_i0_));

            // maximum thrust coefficient according to momentum theory
            ct_max = 2.0 * ( mu_z - lambda_i_ ) * lambda_i_;
        }
        else
        {
            // Johnson: Helicopter Theory, p.106
            // NASA TP-2005-213477, p.14
            lambda_i_ = mu_c_norm * lambda_i0_
                    * ( 0.373*Math::Pow2(mu_c_norm) - 1.991 + 0.598*Math::Pow2(mu_x_norm) );
        }

        UpdateFlappingAnglesAndThrustCoef(mu_x, mu_x2, mu_z, p, q, a_z, gamma);

        if ( ct_ > ct_max ) ct_ = ct_max;
    }

    // induced velocity (Padfield p.117)
    vel_i_  = lambda_i_  * omegaR_;
    vel_i0_ = lambda_i0_ * omegaR_;

    // flapping in axes with sideslip
    double cos_beta_cas = cos(beta_cas);
    double sin_beta_cas = sin(beta_cas);

    double beta_1c_cas = beta_1c_cwas * cos_beta_cas - beta_1s_cwas * sin_beta_cas * ( -1.0 * cdir_ );
    double beta_1s_cas = beta_1s_cwas * cos_beta_cas + beta_1c_cwas * sin_beta_cas * ( -1.0 * cdir_ );

    // flapping coefficients
    beta_1c_ = Math::Satur(-data().beta_max, data().beta_max, beta_1c_cas - theta_1s_);
    beta_1s_ = Math::Satur(-data().beta_max, data().beta_max, beta_1s_cas + theta_1c_);

    coning_angle_ =  beta_0_;
    disk_roll_    = -beta_1s_ * cdir_;
    disk_pitch_   = -beta_1c_;

    // DAS <-> BAS
    das2bas_ = Matrix3x3( Angles( disk_roll_, disk_pitch_, 0.0 ) ).GetTransposed() * ras2bas_;
    bas2das_ = das2bas_.GetTransposed();

    // drag coefficient (Padfield p.98)
    double cd = data().delta_0 + data().delta_2 * Math::Pow2(ct_);

    // H-force coefficient (Bramwell p.100)
    ch_ = 0.5 * data().a * s_ * ( 0.5 * mu_x * cd / data().a
                          + beta_1c_cwas * theta_0_ / 3.0
                          + 0.75 * lambda_ * beta_1c_cwas
                          - 0.5 * mu_x * theta_0_ * lambda_
                          + 0.25 * mu_x * beta_1c_cwas * beta_1c_cwas );

    // torque (Bramwell p.102, Padfield p.115)
    double cqp = cd * s_ * ( 1.0 +3.0 *  mu_x2 ) / 8.0;
    double cqi = -lambda_ * ct_ - mu_x * ch_;
    cq_ = cqp + cqi;
    if ( cq_ > data().cq_max ) cq_ = data().cq_max;

    // Vortex-Ring-State
    if ( vrs_ )
    {
        // Vortex-Ring-State influence
        double kvr = GetVortexRingInfluenceCoef(mu_x_norm, mu_z_norm);
        in_vrs_ = kvr > 0.0;
        ct_ = ct_ * ( 1.0 - kvr*data().vrs_thrust_factor );
        cq_ = cq_ * ( 1.0 + kvr*data().vrs_torque_factor );
    }

    // rotor wake skew angle (Padfield p.121)
    wake_skew_ = atan2(mu_x, lambda_i_ - mu_z);

    // forces and momements
    thrust_ = data().thrust_factor * rho * ar_ * r2_ * omega2_ * ct_;
    hforce_ = data().hforce_factor * rho * ar_ * r2_ * omega2_ * ch_;
    torque_ = data().torque_factor * rho * ar_ * r3_ * omega2_ * cq_;

    f_bas_ = das2bas_  * Vector3(0.0, 0.0, -thrust_)
           + rwas2bas_ * Vector3(hforce_, 0.0, 0.0);
    m_bas_ = ( data().r_hub_bas % f_bas_ )
           + ras2bas_ * Vector3(0.0, 0.0, cdir_ * torque_);

    if ( !f_bas_.IsValid() || !m_bas_.IsValid() )
    {
        f_bas_.Zeroize();
        m_bas_.Zeroize();

        // TODO
    }
}

////////////////////////////////////////////////////////////////////////////////

void MainRotor::Update(double omega,
                       double azimuth,
                       double collective,
                       double cyclicLat,
                       double cyclicLon)
{
    omega_  = omega;
    omega2_ = omega * omega;
    omegaR_ = omega * data().r;

    azimuth_ = azimuth;

    theta_0_  =  collective;
    theta_1c_ = -cyclicLat * cdir_;
    theta_1s_ =  cyclicLon;
}

////////////////////////////////////////////////////////////////////////////////

void MainRotor::UpdateDataDerivedVariables()
{
    bas2ras_ = mc::Matrix3x3(mc::Angles(0.0, data().inclination, 0.0));
    ras2bas_ = bas2ras_.GetTransposed();

    r2_ = data().r * data().r;
    r3_ = data().r * r2_;
    r4_ = data().r * r3_;

    b2_ = data().b * data().b;
    b3_ = data().b * b2_;
    b4_ = data().b * b3_;

    ar_ = M_PI * r2_;
    s_ = ( (double)data().nb ) * data().c * data().r / ar_;

    sb_ = data().blade_mass * data().r  / 2.0;
    ib_ = data().blade_mass * r2_ / 3.0;
    ir_ = data().nb * ib_;

    cdir_ = data().ccw ? 1.0 : -1.0;
}

////////////////////////////////////////////////////////////////////////////////

void MainRotor::UpdateFlappingAnglesAndThrustCoef(double mu_x, double mu_x2, double mu_z,
                                                  double p, double q, double a_z,
                                                  double gamma)
{
    lambda_ = mu_z - lambda_i_;

    // flapping coefficients
    beta_0_ = ( gamma / 2.0 )
            * ( b3_ * lambda_ / 3.0 + cdir_ * b3_ * p * mu_x / ( 6.0 * omega_ ) + b4_ * theta_0_ / 4.0 + b2_ * theta_0_ * mu_x2 / 4.0 )
            - a_z * sb_ / ( ib_ * omega2_ );

    beta_1c_cwas_ = 2.0 * mu_x * ( lambda_ + 4.0 * data().b * theta_0_ / 3.0 ) / ( mu_x2 / 2.0 - b2_ )
            + cdir_ * ( b4_ * p / omega_ - cdir_ * 16.0 * q / ( gamma * omega_ ) ) / ( b2_ * ( mu_x2 / 2.0 - b2_ ) );

    beta_1s_cwas_ = -4.0 * beta_0_ * mu_x * data().b / ( mu_x2 / 2.0 + b2_ ) / 3.0
            + ( b4_ * q / omega_ + cdir_ * 16.0 * p / ( gamma * omega_ ) ) / ( b2_ * ( mu_x2 / 2.0 + b2_ ) );

    // limits
    beta_0_       = Math::Satur(-data().beta_max, data().beta_max, beta_0_);
    beta_1c_cwas_ = Math::Satur(-data().beta_max, data().beta_max, beta_1c_cwas_);
    beta_1s_cwas_ = Math::Satur(-data().beta_max, data().beta_max, beta_1s_cwas_);

    // thrust coefficient
    ct_ = 0.5 * data().a * s_ * data().b * ( lambda_ * data().b / 2.0
                                         + theta_0_ * ( b2_ + 1.5 * mu_x2 ) / 3.0
                                         + data().b * mu_x * beta_1c_cwas_ / 4.0 );
    ct_ = Math::Satur(-data().ct_max, data().ct_max, ct_);
}

////////////////////////////////////////////////////////////////////////////////

void MainRotor::UpdateFlappingAnglesThrustCoefsAndVelocity(double mu_x, double mu_x2, double mu_z,
                                                           double p, double q, double a_z,
                                                           double gamma)
{
    if ( fabs(lambda_i_) < 10e-14 ) lambda_i_ = 10e-14;

    // iteration loop
    for ( unsigned int i = 0; i < n_max; ++i )
    {
        UpdateFlappingAnglesAndThrustCoef(mu_x, mu_x2, mu_z, p, q, a_z, gamma);

        double lambda_i0_new = sqrt(0.5 * ct_);

        if ( IsValid(lambda_i0_new) ) lambda_i0_ = lambda_i0_new;

        // zero function (Padfield p.124)
        // momentum theory - hover and climb
        double lambda_d = mu_x2 + Math::Pow2(lambda_);
        double g_0 = lambda_i_ - ct_ / ( 2.0 * sqrt(lambda_d) );

        // break condition
        if ( fabs( g_0 ) < 1.0e-6 ) break;

        // Newton's iterative scheme
        // (Padfield p.124)
        double h_j = -( 2.0*lambda_i_*sqrt( lambda_d ) - ct_ )*lambda_d
                / ( 2.0*pow(lambda_d, 3.0/2.0) + data().a*s_*lambda_d/4.0 - ct_*lambda_ );

        // (Padfield p.124)
        double f_j = 1.0;
        if ( mu_z > 0.05 ) f_j = 0.6;

        // (Padfield p.124)
        double lambda_i_new = lambda_i_ + f_j * h_j;

        if ( IsValid(lambda_i_new) ) lambda_i_ = lambda_i_new;
    }
}

////////////////////////////////////////////////////////////////////////////////

double MainRotor::GetInGroundEffectThrustCoef(double h_agl)
{
    return 0.0;//mc::getInGroundEffectThrustCoef(h_agl);
}

////////////////////////////////////////////////////////////////////////////////

double MainRotor::GetVortexRingInfluenceCoef(double vx_norm, double vz_norm)
{
    return mc::GetVortexRingInfluenceCoef(vx_norm, vz_norm);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
