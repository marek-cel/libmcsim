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
#ifndef MCSIM_CTRL_HINGEMOMENT_H_
#define MCSIM_CTRL_HINGEMOMENT_H_

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Flight control hinge moment model class.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Etkin B. Dynamics of Atmosferic Flight, 1972, p.222</li>
 *   <li>Paturski Z. Przewodnik po projektach z Mechaniki Lotu, Projekt nr 9: Rownowaga podluzna samolotu i sily na sterownicy wysokosci, p.IX-3. [in Polish]</li>
 * </ul>
 */
class MCSIMAPI HingeMoment
{
public:

    /**
     * @brief Hinge moment model data struct.
     */
    struct Data
    {
        double area  = 0.0;         ///< [m^2] control surface area
        double chord = 0.0;         ///< [m]   control surface mean chord (from hinge to trailing edge)

        double dch_dalpha   = 0.0;  ///< [1/rad] hinge moment coefficient due to angle of attack
        double dch_ddelta   = 0.0;  ///< [1/rad] hinge moment coefficient due to deflection
        double dch_ddelta_t = 0.0;  ///< [1/rad] hinge moment coefficient due to trim deflection
    };

    // LCOV_EXCL_START
    HingeMoment() = default;
    HingeMoment(const HingeMoment&) = delete;
    HingeMoment(HingeMoment&&) = default;
    HingeMoment& operator=(const HingeMoment&) = delete;
    HingeMoment& operator=(HingeMoment&&) = default;
    virtual ~HingeMoment() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes hinge moment.
     * @param dynPress [Pa] dynamic pressure
     * @param alpha   [rad] angle of attack
     * @param delta   [rad] control surface deflection
     * @param delta_t [rad] control surface trim deflection
     */
    virtual double GetHingeMoment(double dynPress, double alpha, double delta,
                                  double delta_t = 0.0) const;

    inline const std::weak_ptr<Data> data() const { return data_; }

protected:

        std::shared_ptr<Data> data_;    ///< hinge moment data struct
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_CTRL_HINGEMOMENT_H_
