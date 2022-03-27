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
#ifndef LIBMCSIM_MASS_VARIABLEMASS_H
#define LIBMCSIM_MASS_VARIABLEMASS_H

////////////////////////////////////////////////////////////////////////////////

#include <mcsim/defs.h>

#include <mcutil/math/Matrix3x3.h>
#include <mcutil/math/Vector3.h>
#include <mcutil/xml/XmlNode.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Variable mass component data.
 *
 * XML configuration file format:
 * @code
 * <variable_mass input="{ variable mass input name }">
 *   <mass_max> { [kg] maximum mass } </mass_max>
 *   <coordinates> { [m] x-coordinate } { [m] y-coordinate } { [m] z-coordinate } </coordinates>
 * </variable_mass>
 * @endcode
 *
 * @see Taylor J.: Classical Mechanics, 2005, p.411
 */
class MCSIMEXPORT VariableMass
{
public:

    /**
     * @brief Reads data.
     * @param dataNode XML node
     */
    void readData( XmlNode &dataNode );

    /** */
    void update( double mass );

    /**
     * @brief Modifies inertia tensor due to variable mass.
     * This function modifies given inertia tensor due to current value
     * of variable mass using Steiner’s theorem threating variable mass
     * as a point mass.
     * @param inertiaTensor [kg*m^2] inertia tensor to be modified expressed in BAS
     */
    void addToInertiaTensor( Matrix3x3 *i_bas );

    /**
     * @brief Modifies first moment of mass due to variable mass.
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

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_MASS_VARIABLEMASS_H
