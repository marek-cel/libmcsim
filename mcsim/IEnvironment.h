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
#ifndef LIBMCSIM_IENVIRONMENT_H
#define LIBMCSIM_IENVIRONMENT_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutil/geo/ECEF.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief The IEnvironment interface class.
 */
class IEnvironment
{
public:

    /**
     * @brief Updates environment model.
     */
    virtual void update() = 0;

    /**
     * @brief Updates environment model.
     * @param ecf ECF position
     */
    virtual void update( const ECEF &ecef ) = 0;

    virtual double getStdRefLevelPressure    () const = 0;
    virtual double getStdRefLevelTemperature () const = 0;
    virtual double getStdRefLevelDensity     () const = 0;

    virtual double getTemperature()  const = 0;
    virtual double getPressure()     const = 0;
    virtual double getDensity()      const = 0;
    virtual double getSpeedOfSound() const = 0;
    virtual double getDynViscosity() const = 0;
    virtual double getKinViscosity() const = 0;

    virtual Vector3 getWind_NED() const = 0;

    virtual Vector3 getRotation_GCS() const = 0;

    virtual Vector3 getGravity_GCS() const = 0;
};

using IEnvironmentPtrS = std::shared_ptr < IEnvironment >;
using IEnvironmentPtrU = std::unique_ptr < IEnvironment >;
using IEnvironmentPtrW = std::weak_ptr   < IEnvironment >;

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_IENVIRONMENT_H
