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

#include <algorithm>
#include <cmath>

#include <mcutils/math/Math.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

double SchrenkDist::GetEllipticWingChord(double y) const
{
    return (4.0 * area_) / (span_ * M_PI) * sqrt(1.0 - std::min(Math::Pow2(2.0 * y / span_), 1.0));
}

////////////////////////////////////////////////////////////////////////////////

double SchrenkDist::GetLiftCoefDist(double y) const
{
    return (GetEllipticWingChord(y) + chord_.GetValue(y)) / area_;
}

////////////////////////////////////////////////////////////////////////////////

void SchrenkDist::set_area(double area)
{
    area_ = area;
}

////////////////////////////////////////////////////////////////////////////////

void SchrenkDist::set_span(double span)
{
    span_ = span;
}

////////////////////////////////////////////////////////////////////////////////

void SchrenkDist::set_chord(const Table& chord)
{
    chord_ = chord;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace mc
