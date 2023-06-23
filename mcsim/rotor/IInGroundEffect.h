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
#ifndef MCSIM_ROTOR_IINGROUNDEFFECT_H_
#define MCSIM_ROTOR_IINGROUNDEFFECT_H_

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief In Ground Effect interface.
 */
class MCSIMAPI IInGroundEffect
{
public:

    // LCOV_EXCL_START
    IInGroundEffect() = default;
    IInGroundEffect(const IInGroundEffect&) = delete;
    IInGroundEffect(IInGroundEffect&&) = default;
    IInGroundEffect& operator=(const IInGroundEffect&) = delete;
    IInGroundEffect& operator=(IInGroundEffect&&) = default;
    virtual ~IInGroundEffect() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Updates in ground effect model.
     * @param vx_vi0 [-] rotor disc plane tangent air velocity normalized by rotor induced velocity in hover
     * @param vz_vi0 [-] rotor disc plane normal air velocity normalized by rotor induced velocity in hover
     */
    virtual void Update( double vx_vi0, double vz_vi0 ) = 0;

    virtual double GetThrustCoef() const = 0;

    virtual double GetTorqueCoef() const = 0;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_ROTOR_IINGROUNDEFFECT_H_
