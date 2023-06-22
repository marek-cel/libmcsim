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

void StabilizerVer::computeForceAndMoment( const Vector3 &vel_air_bas,
                                           const Vector3 &omg_air_bas,
                                           double airDensity )
{
    // stabilizer velocity
    Vector3 vel_stab_bas = vel_air_bas + ( omg_air_bas % _data.r_ac_bas );

    // stabilizer angle of attack and sideslip angle
    double angleOfAttack = GetAngleOfAttack( vel_stab_bas );
    double sideslipAngle = GetSideslipAngle( vel_stab_bas );

    // dynamic pressure
    double dynPress = 0.5 * airDensity * vel_stab_bas.GetLength2();

    Vector3 for_aero( dynPress * getCx( sideslipAngle ) * _data.area,
                      dynPress * getCy( sideslipAngle ) * _data.area,
                      0.0 );

    _for_bas = GetAero2BAS( angleOfAttack, sideslipAngle ) * for_aero;
    _mom_bas = _data.r_ac_bas % _for_bas;

    if ( !_for_bas.IsValid() || !_mom_bas.IsValid() )
    {
        // TODO
    }
}

////////////////////////////////////////////////////////////////////////////////

double StabilizerVer::getCx( double angle ) const
{
    return _data.cx.GetValue( angle );
}

////////////////////////////////////////////////////////////////////////////////

double StabilizerVer::getCy( double angle ) const
{
    return _data.cy.GetValue( angle );
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
