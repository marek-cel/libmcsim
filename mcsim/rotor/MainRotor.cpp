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

#include <mcsim/aero/AeroAngles.h>
#include <mcsim/rotor/RotorUtils.h>

namespace mc {

void MainRotor::UpdateForceAndMoment(const Vector3 & /*vel_bas*/,
                                     const Vector3 &omg_bas,
                                     const Vector3 &acc_bas,
                                     const Vector3 &eps_bas,
                                     const Vector3 &vel_air_bas,
                                     const Vector3 &omg_air_bas,
                                     const Vector3 &grav_bas,
                                     double rho,
                                     double h_agl)
{
    const Data* data = GetData();

    // Lock number
    double gamma = rho * data->a * data->c * _r4 / _ib;

    // RAS <-> CAS
    _ras2cas = Matrix3x3(Angles(_theta_1c, _theta_1s, 0.0));
    _cas2ras = _ras2cas.GetTransposed();

    // BAS <-> CAS
    _bas2cas = _bas2ras * _ras2cas;
    _cas2bas = _bas2cas.GetTransposed();

    // velocity transformations
    Vector3 vel_air_ras = _bas2ras * (vel_air_bas + (omg_air_bas % data->r_hub_bas));
    Vector3 vel_air_cas = _ras2cas * vel_air_ras;

    // sideslip angle
    double beta_cas = GetSideslipAngle(vel_air_cas);
    double beta_ras = GetSideslipAngle(vel_air_ras);

    // RAS -> RWAS
    _ras2rwas = Matrix3x3(Angles(0.0, 0.0, beta_ras));
    _rwas2ras = _ras2rwas.GetTransposed();

    // CAS <-> CWAS
    _cas2cwas = Matrix3x3(Angles(0.0, 0.0, beta_cas));
    _cwas2cas = _cas2cwas.GetTransposed();

    // BAS <-> CWAS
    _bas2cwas = _bas2cas * _cas2cwas;
    _cwas2bas = _bas2cwas.GetTransposed();

    // BAS <-> RWAS
    _bas2rwas = _bas2ras * _ras2rwas;
    _rwas2bas = _bas2rwas.GetTransposed();

    // velocity transformations
    Vector3 vel_air_rwas = _ras2rwas * vel_air_ras;
    Vector3 vel_air_cwas = _cas2cwas * vel_air_cas;
    Vector3 omg_air_cwas = _bas2cwas * omg_air_bas;

    // acceleration
    Vector3 acc_hub_bas = acc_bas
                        + (omg_bas % (omg_bas % data->r_hub_bas))
                        + (eps_bas % data->r_hub_bas)
                        - grav_bas;
    Vector3 acc_hub_cwas = _bas2cwas * acc_hub_bas;

    // angle of attack
    double alpha = GetAngleOfAttack(vel_air_rwas);

    // flapping coefficients
    double beta_1c_cwas = 0.0;
    double beta_1s_cwas = 0.0;

    const double p = omg_air_cwas.p();
    const double q = omg_air_cwas.q();

    const double airspeed = vel_air_cwas.GetLength();

    // rotor advance ratio
    const double mu_x  = airspeed * cos(alpha) / _omegaR;
    const double mu_x2 = mu_x * mu_x;
    const double mu_z  = airspeed * sin(alpha) / _omegaR;

    _ct = 0.0;
    _ch = 0.0;
    _cq = 0.0;

    const double a_z = -acc_hub_cwas.z();

    // hover and climb
    UpdateFlappingAnglesThrustCoefsAndVelocity(mu_x, mu_x2, mu_z, p, q, a_z, gamma);

    double mu_z_norm =  mu_z / _lambda_i0;
    double mu_c_norm = -mu_z_norm;
    double mu_x_norm =  mu_x / _lambda_i0;

    // vortex ring, turbulent wake and windmill
    if ( mu_c_norm < -1.0 )
    {
        double ct_max = _ct;

        if ( mu_c_norm < -2.0 )
        {
            // windmill
            // Padfield: Helicopter Flight Dynamics, p.118
            double mu_z_half = 0.5 * mu_z;
            _lambda_i = mu_z_half - sqrt(mu_z_half*mu_z_half - Math::Pow2(_lambda_i0));

            // maximum thrust coefficient according to momentum theory
            ct_max = 2.0 * (mu_z - _lambda_i) * _lambda_i;
        }
        else
        {
            // Johnson: Helicopter Theory, p.106
            // NASA TP-2005-213477, p.14
            _lambda_i = mu_c_norm * _lambda_i0
                    * (0.373*Math::Pow2(mu_c_norm) - 1.991 + 0.598*Math::Pow2(mu_x_norm));
        }

        UpdateFlappingAnglesAndThrustCoef(mu_x, mu_x2, mu_z, p, q, a_z, gamma);

        if ( _ct > ct_max ) _ct = ct_max;
    }

    // induced velocity (Padfield p.117)
    _vel_i  = _lambda_i  * _omegaR;
    _vel_i0 = _lambda_i0 * _omegaR;

    // flapping in axes with sideslip
    double cos_beta_cas = cos(beta_cas);
    double sin_beta_cas = sin(beta_cas);

    double beta_1c_cas = beta_1c_cwas * cos_beta_cas - beta_1s_cwas * sin_beta_cas * (-1.0 * _cdir);
    double beta_1s_cas = beta_1s_cwas * cos_beta_cas + beta_1c_cwas * sin_beta_cas * (-1.0 * _cdir);

    // flapping coefficients
    _beta_1c = Math::Satur(-data->beta_max, data->beta_max, beta_1c_cas - _theta_1s);
    _beta_1s = Math::Satur(-data->beta_max, data->beta_max, beta_1s_cas + _theta_1c);

    _coning_angle =  _beta_0;
    _disk_roll    = -_beta_1s * _cdir;
    _disk_pitch   = -_beta_1c;

    // DAS <-> BAS
    _das2bas = Matrix3x3(Angles(_disk_roll, _disk_pitch, 0.0)).GetTransposed() * _ras2bas;
    _bas2das = _das2bas.GetTransposed();

    // drag coefficient (Padfield p.98)
    double cd = data->delta_0 + data->delta_2 * Math::Pow2(_ct);

    // H-force coefficient (Bramwell p.100)
    _ch = 0.5 * data->a * _s * (0.5 * mu_x * cd / data->a
        + beta_1c_cwas * _theta_0 / 3.0
        + 0.75 * _lambda * beta_1c_cwas
        - 0.5 * mu_x * _theta_0 * _lambda
        + 0.25 * mu_x * beta_1c_cwas * beta_1c_cwas);

    // torque (Bramwell p.102, Padfield p.115)
    double cqp = cd * _s * (1.0 +3.0 *  mu_x2) / 8.0;
    double cqi = -_lambda * _ct - mu_x * _ch;
    _cq = cqp + cqi;
    if ( _cq > data->cq_max ) _cq = data->cq_max;

    // Vortex-Ring-State
    double c_vsr = GetVortexRingInfluenceCoef(mu_x_norm, mu_z_norm);
    _in_vrs = c_vsr > 0.0;
    _ct = _ct * (1.0 - c_vsr * data->vrs_thrust_factor);
    _cq = _cq * (1.0 + c_vsr * data->vrs_torque_factor);

    // In-Ground-Effect
    double c_ige = GetInGroundEffectThrustCoef(h_agl, airspeed, _vel_i);
    c_ige = Math::Satur(1.0, data->c_ige_max, c_ige);
    _ct = _ct * c_ige * data->ige_thrust_factor;

    // rotor wake skew angle (Padfield p.121)
    _wake_skew = atan2(mu_x, _lambda_i - mu_z);

    // forces and momements
    _thrust = data->thrust_factor * rho * _ar * _r2 * _omega2 * _ct;
    _hforce = data->hforce_factor * rho * _ar * _r2 * _omega2 * _ch;
    _torque = data->torque_factor * rho * _ar * _r3 * _omega2 * _cq;

    _f_bas = _das2bas  * Vector3(0.0, 0.0, -_thrust)
           + _rwas2bas * Vector3(_hforce, 0.0, 0.0);
    _m_bas = (data->r_hub_bas % _f_bas)
           + _ras2bas * Vector3(0.0, 0.0, _cdir * _torque);
}

void MainRotor::Update(double omega,
                       double collective,
                       double cyclicLat,
                       double cyclicLon)
{
    const Data* data = GetData();

    _omega  = omega;
    _omega2 = omega * omega;
    _omegaR = omega * data->r;

    _theta_0  =  collective;
    _theta_1c = -cyclicLat * _cdir;
    _theta_1s =  cyclicLon;
}

void MainRotor::UpdateDataDerivedVariables()
{
    const Data* data = GetData();

    _bas2ras = Matrix3x3(data->a_hub_bas);
    _ras2bas = _bas2ras.GetTransposed();

    _r2 = data->r * data->r;
    _r3 = data->r * _r2;
    _r4 = data->r * _r3;

    _b2 = data->b * data->b;
    _b3 = data->b * _b2;
    _b4 = data->b * _b3;

    _ar = M_PI * _r2;
    _s = (static_cast<double>(data->nb)) * data->c * data->r / _ar;

    _sb = data->blade_mass * data->r  / 2.0;
    _ib = data->blade_mass * _r2 / 3.0;
    _ir = data->nb * _ib;

    _cdir = data->ccw ? 1.0 : -1.0;
}

void MainRotor::UpdateFlappingAnglesAndThrustCoef(double mu_x, double mu_x2, double mu_z,
                                                  double p, double q, double a_z,
                                                  double gamma)
{
    const Data* data = GetData();

    _lambda = mu_z - _lambda_i;

    // flapping coefficients
    _beta_0 = (gamma / 2.0)
        * (_b3 * _lambda / 3.0 + _cdir * _b3 * p * mu_x / (6.0 * _omega) + _b4 * _theta_0 / 4.0 + _b2 * _theta_0 * mu_x2 / 4.0)
        - a_z * _sb / (_ib * _omega2);

    _beta_1c_cwas = 2.0 * mu_x 
        * (_lambda + 4.0 * data->b * _theta_0 / 3.0) / (mu_x2 / 2.0 - _b2)
        + _cdir * (_b4 * p / _omega - _cdir * 16.0 * q / (gamma * _omega)) / (_b2 * ( mu_x2 / 2.0 - _b2));

    _beta_1s_cwas = -4.0 * _beta_0 * mu_x * data->b / (mu_x2 / 2.0 + _b2) / 3.0
        + (_b4 * q / _omega + _cdir * 16.0 * p / (gamma * _omega)) / (_b2 * (mu_x2 / 2.0 + _b2));

    // limits
    _beta_0       = Math::Satur(-data->beta_max, data->beta_max, _beta_0);
    _beta_1c_cwas = Math::Satur(-data->beta_max, data->beta_max, _beta_1c_cwas);
    _beta_1s_cwas = Math::Satur(-data->beta_max, data->beta_max, _beta_1s_cwas);

    // thrust coefficient
    _ct = 0.5 * data->a * _s * data->b
        * (_lambda * data->b / 2.0
        + _theta_0 * (_b2 + 1.5 * mu_x2) / 3.0
        + data->b * mu_x * _beta_1c_cwas / 4.0);
    _ct = Math::Satur(-data->ct_max, data->ct_max, _ct);
}

void MainRotor::UpdateFlappingAnglesThrustCoefsAndVelocity(double mu_x, double mu_x2, double mu_z,
                                                           double p, double q, double a_z,
                                                           double gamma)
{
    const Data* data = GetData();

    if ( fabs(_lambda_i) < 10e-14 ) _lambda_i = 10e-14;

    // iteration loop
    for ( unsigned int i = 0; i < _iter_max; ++i )
    {
        UpdateFlappingAnglesAndThrustCoef(mu_x, mu_x2, mu_z, p, q, a_z, gamma);

        double lambda_i0_new = sqrt(0.5 * _ct);
        if ( IsValid(lambda_i0_new) ) _lambda_i0 = lambda_i0_new;

        // zero function (Padfield p.124)
        // momentum theory - hover and climb
        double lambda_d = mu_x2 + Math::Pow2(_lambda);
        double g_0 = _lambda_i - _ct / (2.0 * sqrt(lambda_d));

        // break condition
        if ( i > 0 && fabs(g_0) < 1.0e-6 ) break;

        // Newton's iterative scheme
        // (Padfield p.124)
        double h_j = -(2.0*_lambda_i*sqrt(lambda_d) - _ct)*lambda_d
            / (2.0*pow(lambda_d, 3.0/2.0) + data->a*_s*lambda_d/4.0 - _ct*_lambda);

        // (Padfield p.124)
        double f_j = 1.0;
        if ( mu_z > 0.05 ) f_j = 0.6;

        // (Padfield p.124)
        double lambda_i_new = _lambda_i + f_j * h_j;

        if ( IsValid(lambda_i_new) ) _lambda_i = lambda_i_new;
    }
}

double MainRotor::GetInGroundEffectThrustCoef(double h_agl, double v, double vi)
{
    return mc::GetInGroundEffectThrustCoef(h_agl, v, vi, _r2);
}

double MainRotor::GetVortexRingInfluenceCoef(double vx_norm, double vz_norm)
{
    return mc::GetVortexRingInfluenceCoef(vx_norm, vz_norm);
}

} // namespace mc
