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

#include <mcsim/rotor/RotorUtils.h>

#include <mcutils/math/Math.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

double GetVortexRingInfluenceCoef(double vx_norm, double vz_norm)
{
    double k0 = 0.0;
    if ( vz_norm > 0.0 && vz_norm < 2.0 )
    {
        k0 = -1.08*vz_norm*vz_norm + 2.04*vz_norm - 0.63;
        k0 = Math::Satur( 0.0, 1.0, k0 );
    }

    double kv = 0.0;
    vx_norm = fabs( vx_norm );
    if ( vx_norm < 1.0 )
    {
        double vx_norm2 = vx_norm * vx_norm;
        double vx_norm3 = vx_norm * vx_norm2;
        double vx_norm4 = vx_norm * vx_norm3;
        double vx_norm5 = vx_norm * vx_norm4;
        kv = 11.25*vx_norm5	- 20.87*vx_norm4 + 9.49*vx_norm3
           -  0.64*vx_norm2 -  0.22*vx_norm  + 1.0;
        kv = Math::Satur( 0.0, 1.0, kv );
    }

    return k0*kv;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
