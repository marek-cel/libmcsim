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

#include <mcsim/aero/StabilizerVer.h>

#include <mcutils/misc/Units.h>

#include <mcsim/aero/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void StabilizerVer::ComputeForceAndMoment(const Vector3 &vel_air_bas,
                                          const Vector3 &omg_air_bas,
                                          double rho)
{
    // stabilizer velocity
    Vector3 vel_stab_bas = vel_air_bas + ( omg_air_bas % data_->r_ac_bas );

    // stabilizer angle of attack and sideslip angle
    double alpha = GetAngleOfAttack(vel_stab_bas);
    double beta  = GetSideslipAngle(vel_stab_bas);

    // dynamic pressure
    double dynPress = 0.5 * rho * vel_stab_bas.GetLength2();

    Vector3 f_aero( dynPress * GetCd(beta) * data_->area,
                    dynPress * GetCy(beta) * data_->area,
                    0.0 );

    Matrix3x3 aero2bas = GetAero2BAS(alpha, beta);

    f_bas_ = aero2bas * f_aero;
    m_bas_ = data_->r_ac_bas % f_bas_;

    if ( !f_bas_.IsValid() || !m_bas_.IsValid() )
    {
        f_bas_.Zeroize();
        m_bas_.Zeroize();

        // TODO
    }
}

////////////////////////////////////////////////////////////////////////////////

double StabilizerVer::GetCd(double beta) const
{
    return data_->cd.GetValue(beta);
}

////////////////////////////////////////////////////////////////////////////////

double StabilizerVer::GetCy(double beta) const
{
    return data_->cy.GetValue(beta);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
