#include "MomentumTheory.h"

#include <cmath>

#ifdef _MSC_VER
#   include <limits>
#endif // _MSC_VER

#include <mcsim/rotor/RotorUtils.h>

void MomentumTheory::Update(double climb_rate, double vi0, double rho)
{
    double mu_c =  climb_rate / omegaR_;
    double mu_d = -climb_rate / omegaR_;

    double lambda_i0 = vi0 / omegaR_;

    // reference values calculated with momentum theory
    double vc_vi0 = climb_rate / vi0;

    // induced velocity
    if ( vc_vi0 >= -1.0 )
    {
        // momentum theory climb
        lambda_i_ = -mu_c / 2.0 + sqrt(pow(mu_c / 2.0, 2.0) + pow(lambda_i0, 2.0));
    }
    else if ( vc_vi0 < -2.0 )
    {
        // momentum theory descend
        lambda_i_ = mu_d / 2.0 - sqrt(pow(mu_d / 2.0, 2.0) - pow(lambda_i0, 2.0));
    }
    else
    {
        // Johnson: Helicopter Theory, p.106
        double mu_c_norm = mu_c / lambda_i0;
        lambda_i_ = mu_c_norm * lambda_i0 * ( 0.373*mu_c_norm*mu_c_norm - 1.991 );
    }
    double vel_i = lambda_i_ * omegaR_;

    // thrust
    thrust_ = std::numeric_limits<double>::quiet_NaN();
    if ( vc_vi0 > -1.0 )
    {
        thrust_ = 2.0 * rho * area_ * ( climb_rate + vel_i ) * vel_i;
    }
    else if ( vc_vi0 < -2.0 )
    {
        thrust_ = 2.0 * rho * area_ * ( fabs( climb_rate ) - vel_i ) * vel_i;
    }

    // Vortex-Ring-State influence
    double kvr = mc::GetVortexRingInfluenceCoef(0.0, mu_d / lambda_i0);
    thrust_ = thrust_ * (1.0 - kvr);
}

void MomentumTheory::set_omega(double omega)
{
    omega_ = omega;
    UpdateDerivedVariables();
}

void MomentumTheory::set_radius(double radius)
{
    radius_ = radius;
    UpdateDerivedVariables();
}

void MomentumTheory::UpdateDerivedVariables()
{
    omegaR_ = omega_ * radius_;
    area_ = M_PI * pow(radius_, 2);
}