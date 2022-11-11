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

#include <mcsim/aero/Fuselage.h>

#include <mcutils/math/Math.h>
#include <mcutils/misc/Units.h>

#include <mcsim/aero/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void Fuselage::computeForceAndMoment( const Vector3 &vel_air_bas,
                                      const Vector3 &omg_air_bas,
                                      double airDensity,
                                      double inducedVelocity,
                                      double wakeSkewAngle )
{
    // rotor downwash on the fuselage, NASA-TM-84281, p.33
    double dwi_dvi = 1.299 + 0.671 * wakeSkewAngle
                   - 1.172 * Math::pow2( wakeSkewAngle )
                   + 0.351 * Math::pow3( wakeSkewAngle );
    double wi = dwi_dvi * inducedVelocity;

    // fuselage velocity
    Vector3 vel_f_bas = vel_air_bas + ( omg_air_bas % _data.r_ac_bas )
                      - Vector3( 0.0, 0.0, -wi );

    // angle of attack and sideslip angle
    _angleOfAttack = getAngleOfAttack( vel_f_bas );
    _sideslipAngle = getSideslipAngle( vel_f_bas );

    // dynamic pressure
    double dynPress = 0.5 * airDensity * vel_f_bas.getLength2();

    Vector3 for_aero( dynPress * getCx( _angleOfAttack ) * _data.area,
                      dynPress * getCy( _sideslipAngle ) * _data.area,
                      dynPress * getCz( _angleOfAttack ) * _data.area );

    Vector3 mom_stab( dynPress * getCl( _sideslipAngle ) * _sl,
                      dynPress * getCm( _angleOfAttack ) * _sl,
                      dynPress * getCn( _sideslipAngle ) * _sl );


    double sinAlpha = sin( _angleOfAttack );
    double cosAlpha = cos( _angleOfAttack );
    double sinBeta  = sin( _sideslipAngle );
    double cosBeta  = cos( _sideslipAngle );

    Vector3 for_bas = getAero2BAS( sinAlpha, cosAlpha, sinBeta, cosBeta ) * for_aero;
    Vector3 mom_bas = getStab2BAS( sinAlpha, cosAlpha ) * mom_stab
                    + ( _data.r_ac_bas % for_bas );

    _for_bas = for_bas;
    _mom_bas = mom_bas;

    if ( !_for_bas.isValid() || !_mom_bas.isValid() )
    {
        // TODO
    }
}

////////////////////////////////////////////////////////////////////////////////

double Fuselage::getCx( double angleOfAttack ) const
{
    return _data.cx.getValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double Fuselage::getCy( double sideslipAngle ) const
{
    return _data.cy.getValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

double Fuselage::getCz( double angleOfAttack ) const
{
    return _data.cz.getValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double Fuselage::getCl( double sideslipAngle ) const
{
    return _data.cl.getValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

double Fuselage::getCm( double angleOfAttack ) const
{
    return _data.cm.getValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double Fuselage::getCn( double sideslipAngle ) const
{
    return _data.cn.getValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
