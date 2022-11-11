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

#include <mcsim/aero/MeanAeroChord.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

double getMeanAerodynamicChord( double cr, double ct )
{
    double tr = ct / cr; // taper ratio
    return ( 2.0 / 3.0 ) * cr * ( 1.0 + tr + tr*tr ) / ( 1.0 + tr );
}

////////////////////////////////////////////////////////////////////////////////

double getMeanAerodynamicChord( const Table &chord )
{
    double sum_mac = 0.0;
    double sum_area = 0.0;

    for ( unsigned int i = 1; i < chord.getSize(); ++i )
    {
        double cr = chord.getValueByIndex( i - 1 );
        double ct = chord.getValueByIndex( i );
        double dy = chord.getKeyByIndex( i ) - chord.getKeyByIndex( i -1 );

        double area = 0.5 * dy * ( cr + ct );

        sum_area += area;
        sum_mac += area * getMeanAerodynamicChord( cr, ct );
    }

    return sum_mac / sum_area;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
