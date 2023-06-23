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
#ifndef MCSIM_ROTOR_ROTORUTILS_H_
#define MCSIM_ROTOR_ROTORUTILS_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Gets Vortex-Ring-State influence coefficient.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Toropov M., Stepanov S.: Modeling of Helicopter Flight Imitation in the Vortex Ring State. 2016</li>
 * </ul>
 *
 * @param vx_norm [-] velcoity tangent to the rotor disc normalized by induced velocity in hover
 * @param vz_norm [-] velcoity perpendiculat to the rotor disc normalized by induced velocity in hover
 * @return Vortex-Ring-State influence coefficient
 */
MCSIMAPI double GetVortexRingInfluenceCoef(double vx_norm, double vz_norm);

MCSIMAPI double GetInGroundEffectThrustCoef(double h_agl, double v, double vi, double r2);

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_ROTOR_ROTORUTILS_H_
