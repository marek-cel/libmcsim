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
#ifndef MCSIM_AERO_WINGBODY_H_
#define MCSIM_AERO_WINGBODY_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Wing-body (tail-off, split-wings) aircraft aerodynamics model class.
 *
 * Forces and moments are calculated, considering different airflow
 * conditions, separately for left and right half wings. Half wing aerodynamic
 * center is considered datum point for computing airflow conditions (airspeed,
 * angle of attack, etc.).
 *
 * @see StabilizerHor
 * @see StabilizerVer
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Drela M.: Flight Vehicle Aerodynamics. 2014. p.203</li>
 * </ul>
 */
class MCSIMAPI WingBody
{
public:

    /**
     * @brief Tail-off aircraft data struct.
     */
    struct Data
    {
        Vector3 r_ac_l_bas;     ///< [m] left half wing aerodynamic center expressed in BAS
        Vector3 r_ac_r_bas;     ///< [m] right half wing aerodynamic center expressed in BAS

        Table cd;               ///< [-] drag coefficient vs [rad] angle of attack
        Table cy;               ///< [-] sideforce coefficient vs [rad] angle of sideslip
        Table cl;               ///< [-] lift coefficient vs [rad] angle of attack
        Table cml;              ///< [-] rolling moment coefficient vs [rad] angle of sideslip
        Table cmm;              ///< [-] pitching moment coefficient vs [rad] angle of attack
        Table cmn;              ///< [-] yawing moment coefficient vs [rad] angle of sideslip

        double area = 0.0;      ///< [m^2] wing reference area
        double mac  = 0.0;      ///< [m] wing mean aerodynamic chord
        double span = 0.0;      ///< [m] wing span
    };

    // LCOV_EXCL_START
    WingBody() = default;
    WingBody(const WingBody&) = delete;
    WingBody(WingBody&&) = default;
    WingBody& operator=(const WingBody&) = delete;
    WingBody& operator=(WingBody&&) = default;
    virtual ~WingBody() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param rho [kg/m^3] air density
     */
    virtual void ComputeForceAndMoment(const Vector3 &vel_air_bas,
                                       const Vector3 &omg_air_bas,
                                       double rho);

    /**
     * @brief Updates wing.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     */
    virtual void Update(const Vector3 &vel_air_bas,
                        const Vector3 &omg_air_bas);

    inline const Vector3& f_bas() const { return f_bas_; }
    inline const Vector3& m_bas() const { return m_bas_; }

    inline bool stall() const { return stall_; }

protected:

    Data data_;                     ///< tail-off aircraft data

    Vector3 f_bas_;                 ///< [N] total force vector expressed in BAS
    Vector3 m_bas_;                 ///< [N*m] total moment vector expressed in BAS

    double area_2_   = 0.0;         ///< [m^2] half wing reference area
    double mac_s_2_  = 0.0;         ///< [m^3] MAC*S/2 where MAC is mean aerodynamic chord and S is reference area
    double span_s_2_ = 0.0;         ///< [m^3] b*S/2 where b is wing span and S is reference area

    double aoa_critical_neg_ = 0.0; ///< [rad] critical angle of attack (negative)
    double aoa_critical_pos_ = 0.0; ///< [rad] critical angle of attack (positive)

    Vector3 vel_l_bas_;             ///< [m/s] left half wing airspeed
    Vector3 vel_r_bas_;             ///< [m/s] right half wing airspeed

    double aoa_l_ = 0.0;            ///< [rad] left half wing angle of attack
    double aoa_r_ = 0.0;            ///< [rad] right half wing angle of attack

    bool stall_ = false;            ///< specifies if wing is stalled

    /**
     * @brief Adds half wing force and moment to the total force and moment.
     * @param r_ac_bas [m] half wing aerodynamic center expressed in BAS
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param rho [kg/m^3] air density
     */
    virtual void AddForceAndMoment(const Vector3 &r_ac_bas,
                                   const Vector3 &vel_air_bas,
                                   const Vector3 &omg_air_bas,
                                   double rho);

    /**
     * @brief Computes drag coefficient.
     * @param alpha [rad] angle of attack
     * @return [-] drag coefficient
     */
    virtual double GetCd(double alpha) const;

    /**
     * @brief Computes sideforce coefficient.
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

#endif // MCSIM_AERO_WINGBODY_H_
