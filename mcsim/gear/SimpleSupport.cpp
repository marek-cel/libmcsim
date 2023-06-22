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

#include <mcsim/gear/SimpleSupport.h>

#include <mcutils/math/Math.h>
#include <mcutils/misc/String.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

SimpleSupport::SimpleSupport( bool staticFriction )
    : _staticFriction ( staticFriction )
{}

////////////////////////////////////////////////////////////////////////////////

void SimpleSupport::computeForceAndMoment( const Vector3 &vel_bas,
                                           const Vector3 &omg_bas,
                                           const Vector3 &r_c_bas,
                                           const Vector3 &n_c_bas,
                                           bool steering, bool antiskid,
                                           double surf_coef )
{
    _for_bas.Zeroize();
    _mom_bas.Zeroize();

    double deflection_norm = n_c_bas * ( r_c_bas - _data.r_u_bas );

    if ( deflection_norm > 1.0e-6 )
    {
        Vector3 dir_lon_bas;
        Vector3 dir_lat_bas;

        double cosDelta = 1.0;
        double sinDelta = 0.0;

        double v_norm = 0.0;
        double v_roll = 0.0;
        double v_slip = 0.0;

        calculateVariables( vel_bas, omg_bas, r_c_bas, n_c_bas, steering,
                            &dir_lon_bas, &dir_lat_bas,
                            &cosDelta, &sinDelta, &v_norm, &v_roll, &v_slip );

        // normal force
        double for_norm = _data.k * deflection_norm - _data.c * v_norm;

        // friction coefs
        double mu_surf_s = _data.mu_s * surf_coef;
        double mu_surf_k = _data.mu_k * surf_coef;

        double mu_roll_t = _data.mu_r;

        double coef_roll = 0.0;
        double coef_slip = 0.0;

        if ( _data.v_max > 0.0 )
        {
            coef_roll = Math::Satur( 0.0, 1.0, fabs( v_roll ) / _data.v_max ) * Math::Sign( v_roll );
            coef_slip = Math::Satur( 0.0, 1.0, fabs( v_slip ) / _data.v_max ) * Math::Sign( v_slip );
        }
        else
        {
            coef_roll = Math::Sign( v_roll );
            coef_slip = Math::Sign( v_slip );
        }

        if ( _staticFriction )
        {
            if ( fabs( _d_roll ) < _data.d_max && fabs( _d_slip ) < _data.d_max )
            {
                // spring-like model of static friction as a logistic function
                double cr = ( 2.0 / ( 1.0 + exp( -3.0 * Math::Satur( 0.0, 1.0, fabs( _d_roll ) / _data.d_max ) ) ) - 1.0 ) * Math::Sign( _d_roll );
                double cs = ( 2.0 / ( 1.0 + exp( -3.0 * Math::Satur( 0.0, 1.0, fabs( _d_slip ) / _data.d_max ) ) ) - 1.0 ) * Math::Sign( _d_slip );

                if      ( coef_roll < 0.0 && cr < 0.0 ) { if ( cr < coef_roll ) coef_roll = cr; }
                else if ( coef_roll > 0.0 && cr > 0.0 ) { if ( cr > coef_roll ) coef_roll = cr; }
                else
                    coef_roll += cr;

                if      ( coef_slip < 0.0 && cs < 0.0 ) { if ( cs < coef_slip ) coef_slip = cs; }
                else if ( coef_slip > 0.0 && cs > 0.0 ) { if ( cs > coef_slip ) coef_slip = cs; }
                else
                    coef_slip += cs;
            }
        }

        coef_roll = Math::Satur( -1.0, 1.0, coef_roll );
        coef_slip = Math::Satur( -1.0, 1.0, coef_slip );

        // braking friction
        mu_roll_t += mu_surf_s * _brake;

        double mu_roll_max = mu_surf_s;

        if ( antiskid )
        {
            mu_roll_max = mu_surf_k + ( mu_surf_s - mu_surf_k ) * ( 1.0 - coef_slip );
        }
        else
        {
            mu_roll_max = mu_surf_k;
        }

        if ( mu_roll_t > mu_roll_max ) mu_roll_t = mu_roll_max;

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

        if ( max_coef < 1.0 && IsValid( max_coef ) )
        {
            for_tan_bas = max_coef * for_tan_bas;
        }

        // resulting forces
        _for_bas = for_tan_bas + for_norm * n_c_bas;
        _mom_bas = r_c_bas % _for_bas;
    }
}

////////////////////////////////////////////////////////////////////////////////

void SimpleSupport::integrate( double timeStep,
                               const Vector3 &vel_bas,
                               const Vector3 &omg_bas,
                               const Vector3 &r_c_bas,
                               const Vector3 &n_c_bas,
                               bool steering )
{
    if ( _staticFriction )
    {
        double deflection_norm = n_c_bas * ( r_c_bas - _data.r_u_bas );

        if ( deflection_norm > 1.0e-6 )
        {
            Vector3 dir_lon_bas;
            Vector3 dir_lat_bas;

            double cosDelta = 1.0;
            double sinDelta = 0.0;

            double v_norm = 0.0;
            double v_roll = 0.0;
            double v_slip = 0.0;

            calculateVariables( vel_bas, omg_bas, r_c_bas, n_c_bas, steering,
                                &dir_lon_bas, &dir_lat_bas,
                                &cosDelta, &sinDelta, &v_norm, &v_roll, &v_slip );

            _d_roll += v_roll * timeStep;
            _d_slip += v_slip * timeStep;

            if ( fabs( v_roll ) > _data.v_max || fabs( v_slip ) > _data.v_max )
            {
                _d_roll = 0.0;
                _d_slip = 0.0;
            }

            if ( fabs( _d_roll ) > _data.d_max ) _d_roll = 0.0;
            if ( fabs( _d_slip ) > _data.d_max ) _d_slip = 0.0;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void SimpleSupport::update( double position, double delta, double brake )
{
    _position = position;
    _delta = delta;
    _brake = brake;
}

////////////////////////////////////////////////////////////////////////////////

void SimpleSupport::calculateVariables( const Vector3 &vel_bas,
                                        const Vector3 &omg_bas,
                                        const Vector3 &r_c_bas,
                                        const Vector3 &n_c_bas,
                                        bool steering,
                                        Vector3 *dir_lon_bas,
                                        Vector3 *dir_lat_bas,
                                        double *cosDelta,
                                        double *sinDelta,
                                        double *v_norm,
                                        double *v_roll,
                                        double *v_slip )
{
    // contact point velocities components
    Vector3 v_c_bas = vel_bas + ( omg_bas % r_c_bas );

    (*v_norm) = n_c_bas * v_c_bas;

    Vector3 v_norm_bas = (*v_norm) * n_c_bas;
    Vector3 v_tang_bas = v_c_bas - v_norm_bas;

    double v_tang = v_tang_bas.GetLength();

    // longitudal and lateral directions
    (*dir_lon_bas) = ( n_c_bas % Vector3::ey() ).GetNormalized();
    (*dir_lat_bas) = ( Vector3::ex() % n_c_bas ).GetNormalized();

    // longitudal and lateral velocity components
    double vel_lon = v_tang_bas * (*dir_lon_bas);
    double vel_lat = v_tang_bas * (*dir_lat_bas);

    // steering
    double delta = 0.0;

    (*cosDelta) = 1.0;
    (*sinDelta) = 0.0;

    if ( _data.steerable && steering )
    {
        delta = Math::Satur( -_data.delta_max, _data.delta_max, _delta );

        (*cosDelta) = cos( delta );
        (*sinDelta) = sin( delta );
    }
    else if ( _data.caster && v_tang > _data.v_max )
    {
        (*cosDelta) =  vel_lon / v_tang;
        (*sinDelta) = -vel_lat / v_tang;

        delta = asin( (*sinDelta) );
    }

    // tire velocities component
    (*v_roll) = vel_lon * (*cosDelta) - vel_lat * (*sinDelta);
    (*v_slip) = vel_lat * (*cosDelta) - vel_lon * (*sinDelta);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
