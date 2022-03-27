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
#ifndef LIBMCSIM_ILANDINGGEAR_H
#define LIBMCSIM_ILANDINGGEAR_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutil/math/Vector3.h>
#include <mcutil/xml/XmlNode.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief The ILandingGear interface class.
 */
class ILandingGear
{
public:

    /**
     * @brief Reads data.
     * @param dataNode XML node
     */
    virtual void readData( XmlNode &dataNode ) = 0;

    virtual void initialize() = 0;

    virtual void update() = 0;

    virtual void computeForceAndMoment() = 0;

    virtual void integrate( double timeStep ) = 0;

    virtual const Vector3& getForce_BAS  () const = 0;
    virtual const Vector3& getMoment_BAS () const = 0;
};

using ILandingGearPtrS = std::shared_ptr < ILandingGear >;
using ILandingGearPtrU = std::unique_ptr < ILandingGear >;
using ILandingGearPtrW = std::weak_ptr   < ILandingGear >;

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_ILANDINGGEAR_H
