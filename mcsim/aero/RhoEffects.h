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
#ifndef MCSIM_AERO_RHOEFFECTS_H_
#define MCSIM_AERO_RHOEFFECTS_H_

#include <mcsim/defs.h>

namespace mc {

/**
 * @brief Returns pressure coefficient factor based on Prandtl-Glauert rule.
 *
 * ### Refernces:
 * - [Prandtl–Glauert singularity - Wikipedia](https://en.wikipedia.org/wiki/Prandtl%E2%80%93Glauert_singularity)
 *
 * @param mach Mach number
 * @param max maximum value
 * @return pressure coefficient factor
 */
double MCSIMAPI GetPrandtlGlauertFactor(double mach, double max = 5.0);

} // namespace mc

#endif // MCSIM_AERO_RHOEFFECTS_H_
