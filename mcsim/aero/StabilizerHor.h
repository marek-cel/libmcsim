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
#ifndef MCSIM_AERO_STABILIZERHOR_H_
#define MCSIM_AERO_STABILIZERHOR_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Horizontal stabilizer aerodynamics model class.
 * This class is intended to be used in tail-off approach.
 * @see Fuselage
 * @see WingBody
 */
class MCSIMAPI StabilizerHor
{
public:

    /**
     * @brief Horizontal stabilizer data struct.
     */
    struct Data
    {
        Vector3 r_ac_bas;           ///< [m] stabilizer aerodynamic center expressed in BAS

        Table cd;                   ///< [-] drag coefficient vs angle of attack
        Table cl;                   ///< [-] lift coefficient vs angle of attack

        Table dw;                   ///< [rad] downwash angle vs wing angle of attack

        double area = 0.0;          ///< [m^2] stabilizer reference area
        double incidence = 0.0;     ///< [rad] stabilizer incidence angle
    };

    // LCOV_EXCL_START
    StabilizerHor() = default;
    StabilizerHor(const StabilizerHor&) = delete;
    StabilizerHor(StabilizerHor&&) = default;
    StabilizerHor& operator=(const StabilizerHor&) = delete;
    StabilizerHor& operator=(StabilizerHor&&) = default;
    virtual ~StabilizerHor() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param rho [kg/m^3] air density
     * @param aoa [rad] wing angle of attack
     */
    virtual void ComputeForceAndMoment(const Vector3& vel_air_bas,
                                       const Vector3& omg_air_bas,
                                       double rho, double aoa = 0.0);

    inline const Data& data() const { return data_; }

    inline const Vector3& f_bas() const { return f_bas_; }
    inline const Vector3& m_bas() const { return m_bas_; }

protected:

    Data data_;                 ///< horizontal stabilizer data struct

    Vector3 f_bas_;             ///< [N] total force vector expressed in BAS
    Vector3 m_bas_;             ///< [N*m] total moment vector expressed in BAS

    /**
     * @brief Computes stabilizer angle of attack.
     * @see Etkin B.: Dynamics of Atmosferic Flight, 1972, p.210
     * @see Raymer D.: Aircraft Design: A Conceptual Approach, 1992, p.426
     * @see Paturski Z.: Przewodnik po projektach z Mechaniki Lotu, Projekt nr 9: Rownowaga podluzna samolotu i sily na sterownicy wysokosci, p.IX-4. [in Polish]
     * @param vel_air_bas [m/s] stabilizer linear velocity relative to the air expressed in BAS
     * @param aoa [rad] wing angle of attack
     * @return [rad] stabilizer angle of attack
     */
    virtual double GetAngleOfAttack(const Vector3& vel_air_bas, double aoa);

    /**
     * @brief Computes drag coefficient.
     * @param alpha [rad] angle of attack
     * @return [-] drag coefficient
     */
    virtual double GetCd(double alpha) const;

    /**
     * @brief Computes lift coefficient.
     * @param alpha [rad] angle of attack
     * @return [-] lift coefficient
     */
    virtual double GetCl(double alpha) const;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_STABILIZERHOR_H_
