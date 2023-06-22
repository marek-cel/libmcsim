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

#include <mcsim/aero/WingBody.h>

#include <mcutils/misc/Units.h>

#include <mcsim/aero/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void WingBody::ComputeForceAndMoment(const Vector3 &vel_air_bas,
                                     const Vector3 &omg_air_bas,
                                     double rho)
{
    f_bas_.Zeroize();
    m_bas_.Zeroize();

    AddForceAndMoment(data_.r_ac_l_bas, vel_air_bas, omg_air_bas, rho);
    AddForceAndMoment(data_.r_ac_r_bas, vel_air_bas, omg_air_bas, rho);

    if ( !f_bas_.IsValid() || !m_bas_.IsValid() )
    {
        // TODO
    }
}

////////////////////////////////////////////////////////////////////////////////

void WingBody::Update(const Vector3 &vel_air_bas, const Vector3 &omg_air_bas)
{
    Vector3 vel_l_bas = vel_air_bas + ( omg_air_bas % data_.r_ac_l_bas );
    Vector3 vel_r_bas = vel_air_bas + ( omg_air_bas % data_.r_ac_r_bas );

    aoa_l_ = GetAngleOfAttack(vel_l_bas);
    aoa_r_ = GetAngleOfAttack(vel_r_bas);

    bool stall_l = ( aoa_l_ < aoa_critical_neg_ ) || ( aoa_l_ > aoa_critical_pos_ );
    bool stall_r = ( aoa_r_ < aoa_critical_neg_ ) || ( aoa_r_ > aoa_critical_pos_ );

    stall_ = stall_l || stall_r;
}

////////////////////////////////////////////////////////////////////////////////

void WingBody::AddForceAndMoment(const Vector3 &r_ac_bas,
                                 const Vector3 &vel_air_bas,
                                 const Vector3 &omg_air_bas,
                                 double rho)
{
    // wing velocity
    Vector3 vel_wing_bas = vel_air_bas + ( omg_air_bas % r_ac_bas );

    // stabilizer angle of attack and sideslip angle
    double alpha = GetAngleOfAttack(vel_wing_bas);
    double beta  = GetSideslipAngle(vel_wing_bas);

    // dynamic pressure
    double dynPress = 0.5 * rho * vel_wing_bas.GetLength2();

    Vector3 f_aero( dynPress * GetCd( alpha ) * area_2_,
                    dynPress * GetCy( beta  ) * area_2_,
                    dynPress * GetCl( alpha ) * area_2_ );

    Vector3 m_stab( dynPress * GetCml( beta  ) * span_s_2_,
                    dynPress * GetCmm( alpha ) * mac_s_2_,
                    dynPress * GetCmn( beta  ) * span_s_2_ );


    double sin_alpha = sin(alpha);
    double cos_alpha = cos(alpha);
    double sin_beta  = sin(beta);
    double cos_beta  = cos(beta);

    Matrix3x3 aero2bas = GetAero2BAS(sin_alpha, cos_alpha, sin_beta, cos_beta);
    Matrix3x3 stab2bas = GetStab2BAS(sin_alpha, cos_alpha);

    Vector3 f_bas = aero2bas * f_aero;
    Vector3 m_bas = stab2bas * m_stab + ( r_ac_bas % f_bas );

    f_bas_ += f_bas;
    m_bas_ += m_bas;
}

////////////////////////////////////////////////////////////////////////////////

double WingBody::GetCd(double alpha) const
{
    return data_.cd.GetValue(alpha);
}

////////////////////////////////////////////////////////////////////////////////

double WingBody::GetCy(double beta) const
{
    return data_.cy.GetValue(beta);
}

////////////////////////////////////////////////////////////////////////////////

double WingBody::GetCl(double alpha) const
{
    return data_.cl.GetValue(alpha);
}

////////////////////////////////////////////////////////////////////////////////

double WingBody::GetCml(double beta) const
{
    return data_.cml.GetValue(beta);
}

////////////////////////////////////////////////////////////////////////////////

double WingBody::GetCmm(double alpha) const
{
    return data_.cmm.GetValue(alpha);
}

////////////////////////////////////////////////////////////////////////////////

double WingBody::GetCmn(double beta) const
{
    return data_.cmn.GetValue(beta);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
