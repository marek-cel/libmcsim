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

#include <mcsim/aero/RhoEffects.h>

#include <mcutils/math/Math.h>
#include <mcutils/misc/Check.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

double getPrandtlGlauertFactor( double machNumber, double max )
{
    double factor  = 1.0;

    if ( machNumber < 1.0 )
    {
        factor = 1.0 / sqrt( fabs( 1.0 - Math::pow2( machNumber ) ) );
    }
    else
    {
        factor = 1.0 / sqrt( fabs( Math::pow2( machNumber ) - 1.0 ) );
    }

    if ( factor > max || !isValid( factor ) )
    {
        factor = max;
    }

    return factor;
}

////////////////////////////////////////////////////////////////////////////////

double getKarmanTsienFactor( double machNumber, double max )
{
    double factor  = 1.0;

    // TODO!

    return factor;
}

////////////////////////////////////////////////////////////////////////////////

double getLaitoneFactor( double machNumber, double max )
{
    double factor  = 1.0;

    // TODO!

    return factor;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
