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
#ifndef LIBMCSIM_IBUILDER_H
#define LIBMCSIM_IBUILDER_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutil/geo/ECEF.h>
#include <mcutil/math/IIntegrator.h>

#include <mcsim/IAircraft.h>

#include <mcsim/IEnvironment.h>

#include <mcsim/IIntersections.h>

#include <mcsim/IControls.h>

#include <mcsim/IAerodynamics.h>
#include <mcsim/ILandingGear.h>
#include <mcsim/IMass.h>
#include <mcsim/IPropulsion.h>

#include <mcsim/IRecorder.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief The IBuilder interface class.
 */
class IBuilder
{
public:

    virtual void setAircraft( const IAircraftPtrS aircraft ) = 0;

    virtual int getStateDimension() = 0;

    virtual IEnvironmentPtrS getEnvironment() = 0;

    virtual IIntersectionsPtrS getIntersections() = 0;

    virtual IControlsPtrS getControls() = 0;

    virtual IAerodynamicsPtrS getAerodynamics () = 0;
    virtual ILandingGearPtrS  getLandingGear  () = 0;
    virtual IMassPtrS         getMass         () = 0;
    virtual IPropulsionPtrS   getPropulsion   () = 0;

    virtual IRecorderPtrS getRecorder() = 0;

    virtual IIntegratorPtrS getIntegrator() = 0;

    virtual ECEF getECEF() = 0;
};

using IBuilderPtrS = std::shared_ptr < IBuilder >;
using IBuilderPtrU = std::unique_ptr < IBuilder >;
using IBuilderPtrW = std::weak_ptr   < IBuilder >;

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_IBUILDER_H
