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

#include <mcsim/mass/PointMass.h>

#include <mcutils/math/Math.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

void PointMass::update( double mass )
{
    _mass = Math::satur( 0.0, _mass_max, mass );
}

////////////////////////////////////////////////////////////////////////////////

void PointMass::addToInertiaTensor( Matrix3x3 *i_bas )
{
    double r_x2 = _pos_bas.x() * _pos_bas.x();
    double r_y2 = _pos_bas.y() * _pos_bas.y();
    double r_z2 = _pos_bas.z() * _pos_bas.z();

    double d_it_xy = _mass * _pos_bas.x() * _pos_bas.y();
    double d_it_xz = _mass * _pos_bas.x() * _pos_bas.z();
    double d_it_yz = _mass * _pos_bas.y() * _pos_bas.z();

    (*i_bas).xx() += _mass * ( r_y2 + r_z2 );
    (*i_bas).xy() -= d_it_xy;
    (*i_bas).xz() -= d_it_xz;

    (*i_bas).yx() -= d_it_xy;
    (*i_bas).yy() += _mass * ( r_x2 + r_z2 );
    (*i_bas).yz() -= d_it_yz;

    (*i_bas).zx() -= d_it_xz;
    (*i_bas).zy() -= d_it_yz;
    (*i_bas).zz() += _mass * ( r_x2 + r_y2 );
}

////////////////////////////////////////////////////////////////////////////////

void PointMass::addToFirstMomentOfMass( Vector3 *s_bas )
{
    (*s_bas) += _mass * _pos_bas;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
