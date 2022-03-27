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
#ifndef LIBMCSIM_IAERODYNAMICS_H
#define LIBMCSIM_IAERODYNAMICS_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutil/math/Vector3.h>
#include <mcutil/xml/XmlNode.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief The IAerodynamics interface class.
 */
class IAerodynamics
{
public:

    /**
     * @brief Reads data.
     * @param dataNode XML node
     */
    virtual void readData( XmlNode &dataNode ) = 0;

    /** @brief Initializes aerodynamics. */
    virtual void initialize() = 0;

    /** @brief Updates aerodynamics. */
    virtual void update() = 0;

    /** @brief Computes force and moment. */
    virtual void computeForceAndMoment() = 0;

    virtual void integrate( double timeStep ) = 0;

    virtual const Vector3& getForce_BAS  () const = 0;
    virtual const Vector3& getMoment_BAS () const = 0;

    /**
     * @brief Returns true if aircraft is stalling, otherwise returns false.
     * @return true if aircraft is stalling, false otherwise
     */
    virtual bool getStall() const = 0;
};

using IAerodynamicsPtrS = std::shared_ptr < IAerodynamics >;
using IAerodynamicsPtrU = std::unique_ptr < IAerodynamics >;
using IAerodynamicsPtrW = std::weak_ptr   < IAerodynamics >;

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_IAERODYNAMICS_H
