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

#include <mcsim/aero/SchrenkDist.h>

#include <cmath>

#include <mcutils/math/Math.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

SchrenkDist::SchrenkDist()
    : _area ( 0.0 )
    , _span ( 0.0 )
    , _4S_bpi ( 0.0 )
    , _2_b ( 0.0 )
{}

////////////////////////////////////////////////////////////////////////////////

SchrenkDist::~SchrenkDist() {}

////////////////////////////////////////////////////////////////////////////////

double SchrenkDist::getDragCoefDist( double y ) const
{
    return ( y < 0.4 * _span ) ? 0.95 : 1.2;
}

////////////////////////////////////////////////////////////////////////////////

double SchrenkDist::getLiftCoefDist( double y ) const
{
    // equivalent elliptical wing chord
    double chord_e = _4S_bpi * sqrt( 1.0 - Math::pow2( y * _2_b ) );
    return 0.5 * ( 1.0 + chord_e / _chord.getValue( y ) );
}

////////////////////////////////////////////////////////////////////////////////

void SchrenkDist::setArea( double area )
{
    _area = area;
    updateAxiliaryParameters();
}

////////////////////////////////////////////////////////////////////////////////

void SchrenkDist::setSpan( double span )
{
    _span = span;
    updateAxiliaryParameters();
}

////////////////////////////////////////////////////////////////////////////////

void SchrenkDist::setChord( const Table &chord )
{
    _chord = chord;
}

////////////////////////////////////////////////////////////////////////////////

void SchrenkDist::updateAxiliaryParameters()
{
    _4S_bpi = ( 4.0 * _area ) / ( _span * M_PI );
    _2_b = 2.0 / _span;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
