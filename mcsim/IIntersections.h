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
#ifndef LIBMCSIM_IINTERSECTIONS_H
#define LIBMCSIM_IINTERSECTIONS_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutil/geo/ECEF.h>
#include <mcutil/math/Vector3.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief The IIntersections interface class.
 */
class IIntersections
{
public:

    /** */
    virtual void update() = 0;

    /** */
    virtual void update( const ECEF &ecef ) = 0;

    /**
     * @brief Returns ground elevation above mean sea level.
     * @param lat [rad] input latitude
     * @param lon [rad] input longitude
     * @return [m] ground elevation above mean sea level
     */
    virtual double getElevation( double lat, double lon ) const = 0;

    /**
     * @brief Gets intersection.
     * @param b [m] beginning of intersection line expressed in WGS
     * @param e [m] end of intersection line expressed in WGS
     * @param r [m] intersection point coordinates expressed in WGS
     * @param n [-] intersection normal vector expressed in WGS
     * @return true on success or false on failure
     */
    virtual bool getIntersection( const Vector3 &b,
                                  const Vector3 &e,
                                  Vector3 *r,
                                  Vector3 *n ) const = 0;

    /**
     * @brief Checks intersection.
     * @param b [m] beginning of intersection line expressed in WGS
     * @param e [m] end of intersection line expressed in WGS
     * @return true if there is an intersection, false otherwise
     */
    virtual bool isIntersection( const Vector3 &b,
                                 const Vector3 &e ) const = 0;

    /**
     * @brief Returns ground normal vector expressed in WGS.
     * @param lat [rad] input latitude
     * @param lon [rad] input longitude
     * @return ground normal vector expressed in WGS
     */
    virtual Vector3 getNormal( double lat, double lon ) const = 0;
};

using IIntersectionsPtrS = std::shared_ptr < IIntersections >;
using IIntersectionsPtrU = std::unique_ptr < IIntersections >;
using IIntersectionsPtrW = std::weak_ptr   < IIntersections >;

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_IINTERSECTIONS_H
