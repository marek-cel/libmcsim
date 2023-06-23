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
#ifndef MCSIM_AERO_STABILIZERVER_H_
#define MCSIM_AERO_STABILIZERVER_H_

////////////////////////////////////////////////////////////////////////////////

#include <memory>

#include <mcutils/math/Table.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Vertical stabilizer aerodynamics model class.
 * This class is intended to be used in tail-off approach.
 * @see Fuselage
 * @see WingBody
 */
class MCSIMAPI StabilizerVer
{
public:

    /**
     * @brief Vertical stabilizer data struct.
     */
    struct Data
    {
        Vector3 r_ac_bas;       ///< [m] stabilizer aerodynamic center expressed in BAS

        Table cd;               ///< [-] drag coefficient vs sideslip angle
        Table cy;               ///< [-] sideforce coefficient vs sideslip angle

        double area = 0.0;      ///< [m^2] stabilizer reference area
    };

    // LCOV_EXCL_START
    StabilizerVer() = default;
    StabilizerVer(const StabilizerVer&) = delete;
    StabilizerVer(StabilizerVer&&) = default;
    StabilizerVer& operator=(const StabilizerVer&) = delete;
    StabilizerVer& operator=(StabilizerVer&&) = default;
    virtual ~StabilizerVer() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param rho [kg/m^3] air density
     */
    virtual void ComputeForceAndMoment(const Vector3& vel_air_bas,
                                       const Vector3& omg_air_bas,
                                       double rho);

    inline const std::shared_ptr<Data> data() const { return data_; }

    inline const Vector3& f_bas() const { return f_bas_; }
    inline const Vector3& m_bas() const { return m_bas_; }

protected:

    std::shared_ptr<Data> data_;    ///< vertical stabilizer data struct

    Vector3 f_bas_;                 ///< [N] total force vector expressed in BAS
    Vector3 m_bas_;                 ///< [N*m] total moment vector expressed in BAS

    /**
     * @brief Computes drag coefficient.
     * @param beta [rad] sideslip angle
     * @return [-] drag coefficient
     */
    virtual double GetCd(double beta) const;

    /**
     * @brief Computes sideforce coefficient.
     * @param beta [rad] sideslip angle
     * @return [-] sideforce coefficient
     */
    virtual double GetCy(double beta) const;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_STABILIZERVER_H_
