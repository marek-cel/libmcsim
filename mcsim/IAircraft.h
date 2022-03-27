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
#ifndef LIBMCSIM_IAIRCRAFT_H
#define LIBMCSIM_IAIRCRAFT_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutil/geo/ECEF.h>

#include <mcutil/math/Angles.h>
#include <mcutil/math/Matrix3x3.h>
#include <mcutil/math/Quaternion.h>
#include <mcutil/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

class MCSIMEXPORT IAircraft
{
public:

    virtual double getTimeStep() const = 0;

    virtual const Vector3&    getPos_GCS() const = 0;
    virtual const Quaternion& getAtt_GCS() const = 0;
    virtual const Vector3&    getVel_BAS() const = 0;
    virtual const Vector3&    getOmg_BAS() const = 0;

    virtual const ECEF& getGCS() const = 0;

    virtual const Matrix3x3& getGCS2BAS() const = 0;
    virtual const Matrix3x3& getBAS2GCS() const = 0;
    virtual const Matrix3x3& getGCS2NED() const = 0;
    virtual const Matrix3x3& getNED2GCS() const = 0;

    virtual const Matrix3x3& getNED2BAS() const = 0;
    virtual const Matrix3x3& getBAS2NED() const = 0;

    virtual const Angles& getAngles_GCS() const = 0;
    virtual const Angles& getAngles_NED() const = 0;

    virtual const Vector3& getVel_NED() const = 0;

    virtual const Vector3& getVel_air_BAS() const = 0;
    virtual const Vector3& getOmg_air_BAS() const = 0;

    virtual const Vector3& getAcc_BAS() const = 0;
    virtual const Vector3& getEps_BAS() const = 0;

    virtual const Vector3& getGrav_GCS() const = 0;
    virtual const Vector3& getGrav_BAS() const = 0;

    virtual const Vector3& getGForceBAS() const = 0;
    virtual const Vector3& getGPilotBAS() const = 0;

    virtual const Vector3& getGround_GCS() const = 0;
    virtual const Vector3& getGround_BAS() const = 0;

    virtual const Vector3& getNormal_GCS() const = 0;
    virtual const Vector3& getNormal_BAS() const = 0;

    virtual double getElevation     () const = 0;
    virtual double getAltitude_ASL  () const = 0;
    virtual double getAltitude_AGL  () const = 0;
    virtual double getRoll          () const = 0;
    virtual double getPitch         () const = 0;
    virtual double getHeading       () const = 0;
    virtual double getAngleOfAttack () const = 0;
    virtual double getSideslipAngle () const = 0;
    virtual double getClimbAngle    () const = 0;
    virtual double getTrackAngle    () const = 0;
    virtual double getSlipSkidAngle () const = 0;
    virtual double getAirspeed      () const = 0;
    virtual double getIAS           () const = 0;
    virtual double getTAS           () const = 0;
    virtual double getGroundSpeed   () const = 0;
    virtual double getDynPress      () const = 0;
    virtual double getMachNumber    () const = 0;
    virtual double getClimbRate     () const = 0;
    virtual double getTurnRate      () const = 0;

    virtual double getDistanceTraveled() const = 0;
};

using IAircraftPtrS = std::shared_ptr < IAircraft >;
using IAircraftPtrU = std::unique_ptr < IAircraft >;
using IAircraftPtrW = std::weak_ptr   < IAircraft >;

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_IAIRCRAFT_H
