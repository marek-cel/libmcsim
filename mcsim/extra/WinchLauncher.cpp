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

#include <mcsim/extra/WinchLauncher.h>

#include <algorithm>
#include <cmath>

#include <mcutils/misc/String.h>
#include <mcutils/physics/Physics.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void WinchLauncher::computeForceAndMoment( const Matrix3x3 &wgs2bas,
                                           const Vector3 &pos_wgs )
{
    _for_bas.zeroize();
    _mom_bas.zeroize();

    if ( _active )
    {
        Vector3 pos_bas = wgs2bas * ( _pos_wgs - pos_wgs );

        double x = std::max( 0.0, pos_bas.getLength() - _len );

        _for_bas = std::max( _for, fabs( x ) * x * _data.stiffness ) * pos_bas.getNormalized();
        _mom_bas = _data.r_a_bas % _for_bas;
    }
}

////////////////////////////////////////////////////////////////////////////////

void WinchLauncher::update( double timeStep,
                            const Matrix3x3 &bas2wgs,
                            const Matrix3x3 &wgs2ned,
                            const Vector3 &pos_wgs,
                            double altitude_agl )
{
    if ( _active )
    {
        if ( timeStep > 0.0 )
        {
            _for = Physics::inertia( _data.for_max, _for, timeStep, _data.tc_for );
            _vel = Physics::inertia( _data.vel_max, _vel, timeStep, _data.tc_vel );
            _len = _len - timeStep * _vel;

            Vector3 pos_ned = wgs2ned * ( pos_wgs - _pos_wgs );

            if ( atan2( -pos_ned.z(), pos_ned.getLengthXY() ) > _data.ang_max )
            {
                _active = false;
            }
        }
        else
        {
            if ( altitude_agl < 0.0 )
            {
                _pos_wgs = pos_wgs + bas2wgs * Vector3( _data.len_max, 0.0, 0.0 );
                _len = _data.len_max;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
