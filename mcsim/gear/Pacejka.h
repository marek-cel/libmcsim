/****************************************************************************//*
 * Copyright (C) 2023 Marek M. Cel
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
#ifndef MCSIM_GEAR_PACEJKA_H_
#define MCSIM_GEAR_PACEJKA_H_

////////////////////////////////////////////////////////////////////////////////

#include <cmath>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Returns Pacejka "Magic Formula" coefficient.
 * @param kappa [-] slip parameter (v_slip/v_roll)
 * @param b b coefficient
 * @param c c coefficient
 * @param d d coefficient
 * @param e e coefficient
 * @return Pacejka "Magic Formula" coefficient
 *
 * ### Refernces:
 * - [Magic Formula tire models - Wikipedia](https://en.wikipedia.org/wiki/Hans_B._Pacejka#Magic_Formula_tire_models)
 * - [Tire-road dynamics given by magic formula coefficients - MATLAB](https://www.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html)
 */
inline double PacejkaFormula(double kappa,
                             double b = 10.0, double c = 1.9,
                             double d = 1.0,  double e = 0.97)
{
    return d * sin(c * atan(b*(1.0 - e)*kappa + e*atan(b*kappa)));
}

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_GEAR_PACEJKA_H_