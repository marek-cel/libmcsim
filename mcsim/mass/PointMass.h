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
#ifndef MCSIM_MASS_POINTMASS_H_
#define MCSIM_MASS_POINTMASS_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Matrix3x3.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Point mass component data.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Taylor J. Classical Mechanics, 2005, p.411</li>
 * </ul>
 */
class MCSIMAPI PointMass
{
public:

    /**
     * @brief Point mass data struct.
     */
    struct Data
    {
        Vector3 r_cg_bas;

        double mass_max = 0.0;        ///< [kg] maximum mass
    };

    /** */
    void Update(double mass);

    /**
     * @brief Modifies inertia tensor due to variable mass.
     * This function modifies given inertia tensor due to current value
     * of variable mass using Steiner’s theorem.
     * @param inertiaTensor [kg*m^2] inertia tensor to be modified expressed in BAS
     */
    void AddToInertiaTensor(Matrix3x3* i_bas);

    /**
     * @brief Modifies first moment of mass due to current mass.
     * @param firstMomentOfMass [kg*m] first mass moment vector to be modified expressed in BAS
     */
    void AddToFirstMomentOfMass(Vector3* s_bas);

    inline const Data& data() const { return data_; }

    inline double mass() const { return mass_; }

private:

    Data data_;         ///< wing runner data

    double mass_;       ///< [kg] current mass
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_MASS_POINTMASS_H_
