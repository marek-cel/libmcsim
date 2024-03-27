#include "MomentumTheory.h"

#include <cmath>

#ifdef _MSC_VER
#   include <limits>
#endif // _MSC_VER

void MomentumTheory::Update(double climb_rate, double vi0, double rho)
{
    double mu_c =  climb_rate / _omegaR;
    double mu_d = -climb_rate / _omegaR;

    double lambda_i0 = vi0 / _omegaR;

    // reference values calculated with momentum theory
    double vc_vi0 = climb_rate / vi0;

    // induced velocity
    if ( vc_vi0 >= -1.0 )
    {
        // momentum theory climb
        _lambda_i = -mu_c / 2.0 + sqrt(pow(mu_c / 2.0, 2.0) + pow(lambda_i0, 2.0));
    }
    else if ( vc_vi0 < -2.0 )
    {
        // momentum theory descend
        _lambda_i = mu_d / 2.0 - sqrt(pow(mu_d / 2.0, 2.0) - pow(lambda_i0, 2.0));
    }
    else
    {
        // Johnson: Helicopter Theory, p.106
        double mu_c_norm = mu_c / lambda_i0;
        _lambda_i = mu_c_norm * lambda_i0 * ( 0.373*mu_c_norm*mu_c_norm - 1.991 );
    }
    double vel_i = _lambda_i * _omegaR;

    // ratio of climb rate to induced velocity at hover
    _vi_vi0 = vel_i / vi0;

    // thrust
    _thrust = std::numeric_limits<double>::quiet_NaN();
    if ( vc_vi0 > -1.0 )
    {
        _thrust = 2.0 * rho * _area * ( climb_rate + vel_i ) * vel_i;
    }
    else if ( vc_vi0 < -2.0 )
    {
        _thrust = 2.0 * rho * _area * ( fabs( climb_rate ) - vel_i ) * vel_i;
    }
}

void MomentumTheory::set_omega(double omega)
{
    _omega = omega;
    UpdateDerivedVariables();
}

void MomentumTheory::set_radius(double radius)
{
    _radius = radius;
    UpdateDerivedVariables();
}

void MomentumTheory::UpdateDerivedVariables()
{
    _omegaR = _omega * _radius;
    _area = M_PI * pow(_radius, 2);
}