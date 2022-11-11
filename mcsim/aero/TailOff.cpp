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

#include <mcsim/aero/TailOff.h>

#include <mcutils/misc/Units.h>

#include <mcsim/aero/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void TailOff::computeForceAndMoment( const Vector3 &vel_air_bas,
                                     const Vector3 &omg_air_bas,
                                     double airDensity )
{
    _for_bas.zeroize();
    _mom_bas.zeroize();

    addForceAndMoment( _data.r_ac_l_bas, vel_air_bas, omg_air_bas, airDensity );
    addForceAndMoment( _data.r_ac_r_bas, vel_air_bas, omg_air_bas, airDensity );

    if ( !_for_bas.isValid() || !_mom_bas.isValid() )
    {
        // TODO
    }
}

////////////////////////////////////////////////////////////////////////////////

void TailOff::update( const Vector3 &vel_air_bas, const Vector3 &omg_air_bas )
{
    Vector3 vel_l_bas = vel_air_bas + ( omg_air_bas % _data.r_ac_l_bas );
    Vector3 vel_r_bas = vel_air_bas + ( omg_air_bas % _data.r_ac_r_bas );

    _aoa_l = getAngleOfAttack( vel_l_bas );
    _aoa_r = getAngleOfAttack( vel_r_bas );

    bool stall_l = ( _aoa_l < _aoa_critical_neg ) || ( _aoa_l > _aoa_critical_pos );
    bool stall_r = ( _aoa_r < _aoa_critical_neg ) || ( _aoa_r > _aoa_critical_pos );

    _stall = stall_l || stall_r;
}

////////////////////////////////////////////////////////////////////////////////

void TailOff::addForceAndMoment( const Vector3 &r_ac_bas,
                                 const Vector3 &vel_air_bas,
                                 const Vector3 &omg_air_bas,
                                 double airDensity )
{
    // wing velocity
    Vector3 vel_wing_bas = vel_air_bas + ( omg_air_bas % r_ac_bas );

    // stabilizer angle of attack and sideslip angle
    double angleOfAttack = getAngleOfAttack( vel_wing_bas );
    double sideslipAngle = getSideslipAngle( vel_wing_bas );

    // dynamic pressure
    double dynPress = 0.5 * airDensity * vel_wing_bas.getLength2();

    Vector3 for_aero( dynPress * getCx( angleOfAttack ) * _area_2,
                      dynPress * getCy( sideslipAngle ) * _area_2,
                      dynPress * getCz( angleOfAttack ) * _area_2 );

    Vector3 mom_stab( dynPress * getCl( sideslipAngle ) * _span_s_2,
                      dynPress * getCm( angleOfAttack ) * _mac_s_2,
                      dynPress * getCn( sideslipAngle ) * _span_s_2 );


    double sinAlpha = sin( angleOfAttack );
    double cosAlpha = cos( angleOfAttack );
    double sinBeta  = sin( sideslipAngle );
    double cosBeta  = cos( sideslipAngle );

    Vector3 for_bas = getAero2BAS( sinAlpha, cosAlpha, sinBeta, cosBeta ) * for_aero;
    Vector3 mom_bas = getStab2BAS( sinAlpha, cosAlpha ) * mom_stab
                    + ( r_ac_bas % for_bas );

    _for_bas += for_bas;
    _mom_bas += mom_bas;
}

////////////////////////////////////////////////////////////////////////////////

double TailOff::getCx( double angleOfAttack ) const
{
    return _data.cx.getValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double TailOff::getCy( double sideslipAngle ) const
{
    return _data.cy.getValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

double TailOff::getCz( double angleOfAttack ) const
{
    return _data.cz.getValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double TailOff::getCl( double sideslipAngle ) const
{
    return _data.cl.getValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

double TailOff::getCm( double angleOfAttack ) const
{
    return _data.cm.getValue( angleOfAttack );
}

////////////////////////////////////////////////////////////////////////////////

double TailOff::getCn( double sideslipAngle ) const
{
    return _data.cn.getValue( sideslipAngle );
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
