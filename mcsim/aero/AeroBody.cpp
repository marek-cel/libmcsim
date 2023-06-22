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

#include <mcsim/aero/AeroBody.h>

#include <mcutils/math/Math.h>
#include <mcutils/misc/Units.h>

#include <mcsim/aero/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void AeroBody::ComputeForceAndMoment(const Vector3 &vel_air_bas,
                                     const Vector3 &omg_air_bas,
                                     double air_dens,
                                     double vel_ind,
                                     double skew_angle)
{
    // rotor downwash on the fuselage, NASA-TM-84281, p.33
    double dwi_dvi = 1.299 + 0.671 * skew_angle
                   - 1.172 * Math::Pow2(skew_angle)
                   + 0.351 * Math::Pow3(skew_angle);
    double wi = dwi_dvi * vel_ind;

    // fuselage velocity
    Vector3 vel_f_bas = vel_air_bas + ( omg_air_bas % data_.r_ac_bas )
                      - Vector3(0.0, 0.0, -wi);

    // angle of attack and sideslip angle
    alpha_ = GetAngleOfAttack(vel_f_bas);
    beta_  = GetSideslipAngle(vel_f_bas);

    // dynamic pressure
    double dyn_press = 0.5 * air_dens * vel_f_bas.GetLength2();

    Vector3 f_aero( dyn_press * GetCx( alpha_ ) * data_.area,
                    dyn_press * GetCy( beta_  ) * data_.area,
                    dyn_press * GetCz( alpha_ ) * data_.area );

    Vector3 m_stab( dyn_press * GetCl( beta_  ) * sl_,
                    dyn_press * GetCm( alpha_ ) * sl_,
                    dyn_press * GetCn( beta_  ) * sl_ );


    double sin_alpha = sin( alpha_ );
    double cos_alpha = cos( alpha_ );
    double sin_beta  = sin( beta_  );
    double cos_beta  = cos( beta_  );

    Vector3 f_bas = GetAero2BAS(sin_alpha, cos_alpha, sin_beta, cos_beta) * f_aero;
    Vector3 m_bas = GetStab2BAS(sin_alpha, cos_alpha) * m_stab
                  + ( data_.r_ac_bas % f_bas );

    f_bas_ = f_bas;
    m_bas_ = m_bas;

    if ( !f_bas_.IsValid() || !m_bas_.IsValid() )
    {
        // TODO
    }
}

////////////////////////////////////////////////////////////////////////////////

double AeroBody::GetCx( double angleOfAttack ) const
{
    return data_.cx.GetValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double AeroBody::GetCy( double sideslipAngle ) const
{
    return data_.cy.GetValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

double AeroBody::GetCz( double angleOfAttack ) const
{
    return data_.cz.GetValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double AeroBody::GetCl( double sideslipAngle ) const
{
    return data_.cl.GetValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

double AeroBody::GetCm( double angleOfAttack ) const
{
    return data_.cm.GetValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double AeroBody::GetCn( double sideslipAngle ) const
{
    return data_.cn.GetValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
