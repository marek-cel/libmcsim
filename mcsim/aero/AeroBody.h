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
#ifndef MCSIM_AERO_AEROBODY_H_
#define MCSIM_AERO_AEROBODY_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Body aerodynamics model class.
 *
 * Very simple aerodynamic forces and moments model. It can be used to represent
 * bodies that have a relatively small impact on overall aerodynamic forces and
 * moments, such as the fuselage, nacelles, pods, and fairings.
 *
 * In addition to the basic functionality, the model could take into account
 * effects due to rotorcraft lift rotor downwash.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Fiszdon W.: Mechanika Lotu, Czesc I, 1961, p. 44-52. [in Polish]</li>
 *   <li><a href="https://ntrs.nasa.gov/citations/19830001781">A Mathematical Model of a Single Main Rototr Helicopter for Piloted Simulation. NASA-TM-84281</a></li>
 * </ul>
 */
class MCSIMAPI AeroBody
{
public:

    /**
     * @brief Aero body data struct.
     */
    struct Data
    {
        Vector3 r_ac_bas;       ///< [m] fuselage aerodynamic center expressed in BAS

        Table cd;               ///< [-] drag coefficient vs [rad] angle of attack in wind axes
        Table cy;               ///< [-] side force coefficient vs [rad] angle of sideslip in wind axes
        Table cl;               ///< [-] lift coefficient vs [rad] angle of attack in wind axes

        Table cml;              ///< [-] rolling moment coefficient vs [rad] angle of sideslip in stability axes
        Table cmm;              ///< [-] pitching moment coefficient vs [rad] angle of attack in stability axes
        Table cmn;              ///< [-] yawing moment coefficient vs [rad] angle of sideslip in stability axes

        double length = 0.0;    ///< [m] reference length
        double area   = 0.0;    ///< [m^2] reference area
    };

    // LCOV_EXCL_START
    AeroBody() = default;
    AeroBody(const AeroBody&) = delete;
    AeroBody(AeroBody&&) = default;
    AeroBody& operator=(const AeroBody&) = delete;
    AeroBody& operator=(AeroBody&&) = default;
    virtual ~AeroBody() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param rho [kg/m^3] air density
     * @param vel_ind [m/s] rotor induced velocity (for rotorcrafts)
     * @param skew_ang [rad] rotor wake skew angle (for rotorcrafts)
     */
    virtual void ComputeForceAndMoment(const Vector3& vel_air_bas,
                                       const Vector3& omg_air_bas,
                                       double rho,
                                       double vel_ind = 0.0,
                                       double skew_ang = 0.0);

    inline const Vector3& f_bas() const { return f_bas_; }
    inline const Vector3& m_bas() const { return m_bas_; }

protected:

    Data data_;                 ///< aero body data struct

    Vector3 f_bas_;             ///< [N] total force vector expressed in BAS
    Vector3 m_bas_;             ///< [N*m] total moment vector expressed in BAS

    double sl_;                 ///< [m^3] S*l where S is reference area and l is reference length

    double alpha_;              ///< [rad] angle of attack
    double beta_;               ///< [rad] angle of sideslip

    /**
     * @brief Computes drag coefficient.
     * @param alpha [rad] angle of attack
     * @return [-] drag coefficient
     */
    virtual double GetCd(double alpha) const;

    /**
     * @brief Computes side force coefficient.
     * @param beta [rad] angle of sideslip
     * @return [-] sideforce coefficient
     */
    virtual double GetCy(double beta) const;

    /**
     * @brief Computes lift coefficient.
     * @param alpha [rad] angle of attack
     * @return [-] lift coefficient
     */
    virtual double GetCl(double alpha) const;

    /**
     * @brief Computes rolling moment coefficient.
     * @param beta [rad] angle of sideslip
     * @return [-] rolling moment coefficient
     */
    virtual double GetCml(double beta) const;

    /**
     * @brief Computes pitching moment coefficient.
     * @param alpha [rad] angle of attack
     * @return [-] pitching moment coefficient
     */
    virtual double GetCmm(double alpha) const;

    /**
     * @brief Computes yawing moment coefficient.
     * @param beta [rad] angle of sideslip
     * @return [-] yawing moment coefficient
     */
    virtual double GetCmn(double beta) const;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_AEROBODY_H_
