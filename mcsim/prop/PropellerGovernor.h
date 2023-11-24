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
#ifndef MCSIM_PROP_PROPELLERGOVERNOR_H_
#define MCSIM_PROP_PROPELLERGOVERNOR_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Propeller governor model class.
 */
class MCSIMAPI PropellerGovernor
{
public:

    /** Governor data struct. */
    struct Data
    {
        Table prop_rpm;         ///< [rpm] propeller setpoint RPM vs [-] propeller RPM lever position

        double gain_1 = 0.0;    ///< first order gain
        double gain_2 = 0.0;    ///< second order gain
    };

    // LCOV_EXCL_START
    PropellerGovernor() = default;
    PropellerGovernor(const PropellerGovernor&) = delete;
    PropellerGovernor(PropellerGovernor&&) = default;
    PropellerGovernor& operator=(const PropellerGovernor&) = delete;
    PropellerGovernor& operator=(PropellerGovernor&&) = default;
    virtual ~PropellerGovernor() = default;
    // LCOV_EXCL_STOP

    virtual void Update(double prop_lever, double rpm);

    virtual const Data& data() const = 0;

    inline double pitch() const { return pitch_; }

protected:

    double pitch_ = 0.0;    ///< <0.0;1.0> normalized propeller pitch
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_PROP_PROPELLERGOVERNOR_H_
