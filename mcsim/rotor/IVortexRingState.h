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
#ifndef MCSIM_ROTOR_IVORTEXRINGSTATE_H_
#define MCSIM_ROTOR_IVORTEXRINGSTATE_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Vortex Ring State interface.
 */
class MCSIMAPI IVortexRingState
{
public:

    // LCOV_EXCL_START
    IVortexRingState() = default;
    IVortexRingState(const IVortexRingState&) = delete;
    IVortexRingState(IVortexRingState&&) = default;
    IVortexRingState& operator=(const IVortexRingState&) = delete;
    IVortexRingState& operator=(IVortexRingState&&) = default;
    virtual ~IVortexRingState() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Updates vortex ring state model.
     * @param vx_vi0 [-] rotor disc plane tangent air velocity normalized by rotor induced velocity in hover
     * @param vz_vi0 [-] rotor disc plane normal air velocity normalized by rotor induced velocity in hover
     */
    virtual void Update(double vx_vi0, double vz_vi0) = 0;

    virtual double GetThrustCoef() const = 0;

    virtual double GetTorqueCoef() const = 0;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_ROTOR_IVORTEXRINGSTATE_H_
