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

#include <mcsim/gear/SimpleGear.h>

#include <mcutils/math/Math.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void SimpleGear::UpdateForceAndMoment(const Vector3& vel_bas,
                                      const Vector3& omg_bas,
                                      const Vector3& r_c_bas,
                                      const Vector3& n_c_bas,
                                      bool steering,
                                      bool antiskid,
                                      double surf_coef)
{
    f_bas_.Zeroize();
    m_bas_.Zeroize();

    double deflection_norm = n_c_bas * (r_c_bas - data().r_u_bas);

    if ( deflection_norm > 1.0e-6 )
    {
        Vector3 dir_lon_bas;
        Vector3 dir_lat_bas;

        double cosDelta = 1.0;
        double sinDelta = 0.0;

        double v_norm = 0.0;
        double v_roll = 0.0;
        double v_slip = 0.0;

        CalculateVariables(vel_bas, omg_bas, r_c_bas, n_c_bas, steering,
                           &dir_lon_bas, &dir_lat_bas,
                           &cosDelta, &sinDelta, &v_norm, &v_roll, &v_slip);

        // normal force
        double for_norm = data().k * deflection_norm - data().c * v_norm;

        // friction coefs
        double mu_surf_s = data().mu_s * surf_coef;
        double mu_surf_k = data().mu_k * surf_coef;

        double mu_roll_t = data().mu_r;

        double coef_roll = 0.0;
        double coef_slip = 0.0;

        if ( data().v_max > 0.0 )
        {
            coef_roll = Math::Satur(0.0, 1.0, fabs(v_roll) / data().v_max) * Math::Sign(v_roll);
            coef_slip = Math::Satur(0.0, 1.0, fabs(v_slip) / data().v_max) * Math::Sign(v_slip);
        }
        else
        {
            coef_roll = Math::Sign(v_roll);
            coef_slip = Math::Sign(v_slip);
        }

        if ( stat_friction_ )
        {
            if ( fabs(d_roll_) < data().d_max && fabs(d_slip_) < data().d_max )
            {
                // spring-like model of static friction as a logistic function
                double cr = (2.0 / (1.0 + exp(-3.0 * Math::Satur(0.0, 1.0, fabs(d_roll_) / data().d_max))) - 1.0) * Math::Sign(d_roll_);
                double cs = (2.0 / (1.0 + exp(-3.0 * Math::Satur(0.0, 1.0, fabs(d_slip_) / data().d_max))) - 1.0) * Math::Sign(d_slip_);

                if ( coef_roll < 0.0 && cr < 0.0 )
                { 
                    if ( cr < coef_roll ) coef_roll = cr; 
                }
                else if ( coef_roll > 0.0 && cr > 0.0 )
                { 
                    if ( cr > coef_roll ) coef_roll = cr; 
                }
                else
                {
                    coef_roll += cr;
                }

                if ( coef_slip < 0.0 && cs < 0.0 )
                { 
                    if ( cs < coef_slip ) coef_slip = cs; 
                }
                else if ( coef_slip > 0.0 && cs > 0.0 )
                { 
                    if ( cs > coef_slip ) coef_slip = cs; 
                }
                else
                {
                    coef_slip += cs;
                }
            }
        }

        coef_roll = Math::Satur(-1.0, 1.0, coef_roll);
        coef_slip = Math::Satur(-1.0, 1.0, coef_slip);

        // braking friction
        mu_roll_t += mu_surf_s * brake_;

        double mu_roll_max = mu_surf_s;

        if ( antiskid )
        {
            mu_roll_max = mu_surf_k + (mu_surf_s - mu_surf_k) * (1.0 - coef_slip);
        }
        else
        {
            mu_roll_max = mu_surf_k;
        }

        if ( mu_roll_t > mu_roll_max )
        {
            mu_roll_t = mu_roll_max;
        }

        // tire forces
        double for_norm_pos = ( for_norm < 0.0 ) ? 0.0 : for_norm;
        double for_roll_max = for_norm_pos * mu_roll_t;
        double for_slip_max = for_norm_pos * mu_surf_k;

        // continuous friction model
        double for_roll = for_roll_max * coef_roll;
        double for_slip = for_slip_max * coef_slip;

        // tangent forces
        double for_lon = -for_roll * cosDelta + for_slip * sinDelta;
        double for_lat = -for_slip * cosDelta + for_roll * sinDelta;

        // preliminary tangent force
        Vector3 for_tan_bas = for_lon * dir_lon_bas + for_lat * dir_lat_bas;

        // max friction check
        double max_fric = mu_surf_s * for_norm;
        double max_coef = max_fric / for_tan_bas.GetLength();

        if ( max_coef < 1.0 && IsValid(max_coef) )
        {
            for_tan_bas = max_coef * for_tan_bas;
        }

        // resulting forces
        f_bas_ = for_tan_bas + for_norm * n_c_bas;
        m_bas_ = r_c_bas % f_bas_;
    }
}

////////////////////////////////////////////////////////////////////////////////

void SimpleGear::Integrate(double dt,
                           const Vector3 &vel_bas,
                           const Vector3 &omg_bas,
                           const Vector3 &r_c_bas,
                           const Vector3 &n_c_bas,
                           bool steering)
{
    if ( stat_friction_ )
    {
        double deflection_norm = n_c_bas * (r_c_bas - data().r_u_bas);

        if ( deflection_norm > 1.0e-6 )
        {
            Vector3 dir_lon_bas;
            Vector3 dir_lat_bas;

            double cosDelta = 1.0;
            double sinDelta = 0.0;

            double v_norm = 0.0;
            double v_roll = 0.0;
            double v_slip = 0.0;

            CalculateVariables(vel_bas, omg_bas, r_c_bas, n_c_bas, steering,
                               &dir_lon_bas, &dir_lat_bas,
                               &cosDelta, &sinDelta, &v_norm, &v_roll, &v_slip);

            d_roll_ += v_roll * dt;
            d_slip_ += v_slip * dt;

            if ( fabs(v_roll) > data().v_max || fabs(v_slip) > data().v_max )
            {
                d_roll_ = 0.0;
                d_slip_ = 0.0;
            }

            if ( fabs(d_roll_) > data().d_max ) d_roll_ = 0.0;
            if ( fabs(d_slip_) > data().d_max ) d_slip_ = 0.0;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void SimpleGear::Update(double position, double delta, double brake)
{
    position_ = position;
    delta_ = delta;
    brake_ = brake;
}

////////////////////////////////////////////////////////////////////////////////

void SimpleGear::CalculateVariables(const Vector3& vel_bas,
                                    const Vector3& omg_bas,
                                    const Vector3& r_c_bas,
                                    const Vector3& n_c_bas,
                                    bool steering,
                                    Vector3* dir_lon_bas,
                                    Vector3* dir_lat_bas,
                                    double* cos_delta,
                                    double* sin_delta,
                                    double* v_norm,
                                    double* v_roll,
                                    double* v_slip)
{
    // contact point velocities components
    Vector3 v_c_bas = vel_bas + (omg_bas % r_c_bas);

    *v_norm = n_c_bas * v_c_bas;

    Vector3 v_norm_bas = (*v_norm) * n_c_bas;
    Vector3 v_tang_bas = v_c_bas - v_norm_bas;

    double v_tang = v_tang_bas.GetLength();

    // longitudal and lateral directions
    *dir_lon_bas = (n_c_bas % Vector3::ey()).GetNormalized();
    *dir_lat_bas = (Vector3::ex() % n_c_bas).GetNormalized();

    // longitudal and lateral velocity components
    double vel_lon = v_tang_bas * (*dir_lon_bas);
    double vel_lat = v_tang_bas * (*dir_lat_bas);

    // steering
    double delta = 0.0;

    *cos_delta = 1.0;
    *sin_delta = 0.0;

    if ( data().steerable && steering )
    {
        delta = Math::Satur(-data().delta_max, data().delta_max, delta_);

        *cos_delta = cos(delta);
        *sin_delta = sin(delta);
    }
    else if ( data().caster && v_tang > data().v_max )
    {
        *cos_delta =  vel_lon / v_tang;
        *sin_delta = -vel_lat / v_tang;

        delta = asin(*sin_delta);
    }

    // tire velocities component
    *v_roll = vel_lon * (*cos_delta) - vel_lat * (*sin_delta);
    *v_slip = vel_lat * (*cos_delta) - vel_lon * (*sin_delta);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
