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
 * @brief Gets Vortex-Ring-State intensity for the axial flow.
 *
 * ### Refernces:
 * - Toropov M., Stepanov S.: Modeling of Helicopter Flight Imitation in the Vortex Ring State. 2016
 *
 * @param vz_norm [-] velcoity perpendiculat to the rotor disc normalized by induced velocity in hover
 * @return Vortex-Ring-State intensity for the axial flow
 */
MCSIMAPI double GetVortexRingK0(double vz_norm);

/**
 * @brief Gets Vortex-Ring-State horizontal speed effect coefficient.
 *
 * ### Refernces:
 * - Toropov M., Stepanov S.: Modeling of Helicopter Flight Imitation in the Vortex Ring State. 2016
 *
 * @param vx_norm [-] velcoity tangent to the rotor disc normalized by induced velocity in hover
 * @return Vortex-Ring-State horizontal speed effect coefficient
 */
MCSIMAPI double GetVortexRingKv(double vx_norm);

/**
 * @brief Gets Vortex-Ring-State influence coefficient.
 *
 * ### Refernces:
 * - Toropov M., Stepanov S.: Modeling of Helicopter Flight Imitation in the Vortex Ring State. 2016
 *
 * @param vx_norm [-] velcoity tangent to the rotor disc normalized by induced velocity in hover
 * @param vz_norm [-] velcoity perpendiculat to the rotor disc normalized by induced velocity in hover
 * @return Vortex-Ring-State influence coefficient
 */
MCSIMAPI inline double GetVortexRingInfluenceCoef(double vx_norm, double vz_norm)
{
    return GetVortexRingK0(vz_norm)*GetVortexRingKv(vx_norm);
}

/**
 * @brief Gets In-Ground-Effect thrust coefficient.
 * 
 * ### References:
 * - Padfield G.: Helicopter Flight Dynamics, 2007
 * 
 * @param h_agl [m]   altitude above ground level
 * @param v     [m/s] rotor hub airspeed
 * @param vi    [m/s] rotor induced velocity
 * @param r2    [m^2] rotor radius squared
 * @return In-Ground-Effect thrust coefficient
 */
MCSIMAPI double GetInGroundEffectThrustCoef(double h_agl, double v, double vi, double r2);

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_ROTOR_ROTORUTILS_H_
