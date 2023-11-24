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

void PointMass::Update(double mass)
{
    mass_ = Math::Satur(0.0, data().mass_max, mass);
}

////////////////////////////////////////////////////////////////////////////////

void PointMass::AddToInertiaTensor(Matrix3x3* i_bas)
{
    double r_x2 = data().r_cg_bas.x() * data().r_cg_bas.x();
    double r_y2 = data().r_cg_bas.y() * data().r_cg_bas.y();
    double r_z2 = data().r_cg_bas.z() * data().r_cg_bas.z();

    double d_it_xy = mass_ * data().r_cg_bas.x() * data().r_cg_bas.y();
    double d_it_xz = mass_ * data().r_cg_bas.x() * data().r_cg_bas.z();
    double d_it_yz = mass_ * data().r_cg_bas.y() * data().r_cg_bas.z();

    (*i_bas).xx() += mass_ * ( r_y2 + r_z2 );
    (*i_bas).xy() -= d_it_xy;
    (*i_bas).xz() -= d_it_xz;

    (*i_bas).yx() -= d_it_xy;
    (*i_bas).yy() += mass_ * ( r_x2 + r_z2 );
    (*i_bas).yz() -= d_it_yz;

    (*i_bas).zx() -= d_it_xz;
    (*i_bas).zy() -= d_it_yz;
    (*i_bas).zz() += mass_ * ( r_x2 + r_y2 );
}

////////////////////////////////////////////////////////////////////////////////

void PointMass::AddToFirstMomentOfMass(Vector3* s_bas)
{
    *s_bas += mass_ * data().r_cg_bas;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
