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

    /** */
    void update( double mass );

    /**
     * @brief Modifies inertia tensor due to variable mass.
     * This function modifies given inertia tensor due to current value
     * of variable mass using Steiner’s theorem.
     * @param inertiaTensor [kg*m^2] inertia tensor to be modified expressed in BAS
     */
    void addToInertiaTensor( Matrix3x3 *i_bas );

    /**
     * @brief Modifies first moment of mass due to current mass.
     * @param firstMomentOfMass [kg*m] first mass moment vector to be modified expressed in BAS
     */
    void addToFirstMomentOfMass( Vector3 *s_bas );

    /**
     * @brief Returns current mass.
     * @return [kg] current mass
     */
    inline double getMass() const { return _mass; }

private:

    Vector3 _pos_bas;       ///< [m] position expressed in BAS

    double _mass_max;       ///< [kg] maximum mass
    double _mass;           ///< [kg] current mass
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_MASS_POINTMASS_H_
