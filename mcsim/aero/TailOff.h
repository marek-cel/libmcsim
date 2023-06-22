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
#ifndef MCSIM_AERO_TAILOFF_H_
#define MCSIM_AERO_TAILOFF_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Tail-off aircraft (wing-body) aerodynamics model class.
 *
 * Forces and moments are calculated, considering different airflow
 * conditions, separately for left and right half wings. Half wing aerodynamic
 * center is considered datum point for computing airflow conditions (airspeed,
 * angle of attack, etc.).
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Drela M.: Flight Vehicle Aerodynamics. 2014. p.203</li>
 * </ul>
 */
class MCSIMAPI TailOff
{
public:

    /**
     * @brief Tail-off aircraft data struct.
     */
    struct Data
    {
        Vector3 r_ac_l_bas;     ///< [m] left half wing aerodynamic center expressed in BAS
        Vector3 r_ac_r_bas;     ///< [m] right half wing aerodynamic center expressed in BAS

        Table cx;               ///< [-] drag coefficient vs [rad] angle of attack
        Table cy;               ///< [-] sideforce coefficient vs [rad] angle of sideslip
        Table cz;               ///< [-] lift coefficient vs [rad] angle of attack
        Table cl;               ///< [-] rolling moment coefficient vs [rad] angle of sideslip
        Table cm;               ///< [-] pitching moment coefficient vs [rad] angle of attack
        Table cn;               ///< [-] yawing moment coefficient vs [rad] angle of sideslip

        double area = 0.0;      ///< [m^2] wing reference area
        double mac  = 0.0;      ///< [m] wing mean aerodynamic chord
        double span = 0.0;      ///< [m] wing span
    };

    // LCOV_EXCL_START
    // excluded from coverage report due to deleting destructor calling issues
    /** @brief Destructor. */
    virtual ~TailOff() = default;
    // LCOV_EXCL_STOP

    /**
     * @brief Computes force and moment.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param airDensity [kg/m^3] air density
     */
    virtual void computeForceAndMoment( const Vector3 &vel_air_bas,
                                        const Vector3 &omg_air_bas,
                                        double airDensity );

    /**
     * @brief Updates wing.
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     */
    virtual void update( const Vector3 &vel_air_bas,
                         const Vector3 &omg_air_bas );

    inline const Vector3& getForce_BAS  () const { return _for_bas; }
    inline const Vector3& getMoment_BAS () const { return _mom_bas; }

    inline bool getStall() const { return _stall; }

protected:

    Data _data;         ///< tail-off aircraft data

    Vector3 _for_bas;   ///< [N] total force vector expressed in BAS
    Vector3 _mom_bas;   ///< [N*m] total moment vector expressed in BAS

    double _area_2   { 0.0 };           ///< [m^2] half wing reference area
    double _mac_s_2  { 0.0 };           ///< [m^3] MAC*S/2 where MAC is mean aerodynamic chord and S is reference area
    double _span_s_2 { 0.0 };           ///< [m^3] b*S/2 where b is wing span and S is reference area

    double _aoa_critical_neg { 0.0 };   ///< [rad] critical angle of attack (negative)
    double _aoa_critical_pos { 0.0 };   ///< [rad] critical angle of attack (positive)

    Vector3 _vel_l_bas;                 ///< [m/s] left half wing airspeed
    Vector3 _vel_r_bas;                 ///< [m/s] right half wing airspeed

    double _aoa_l { 0.0 };              ///< [rad] left half wing angle of attack
    double _aoa_r { 0.0 };              ///< [rad] right half wing angle of attack

    bool _stall { false };              ///< specifies if wing is stalled

    /**
     * @brief Adds half wing force and moment to the total force and moment.
     * @param r_ac_bas [m] half wing aerodynamic center expressed in BAS
     * @param vel_air_bas [m/s] aircraft linear velocity relative to the air expressed in BAS
     * @param omg_air_bas [rad/s] aircraft angular velocity relative to the air expressed in BAS
     * @param airDensity [kg/m^3] air density
     */
    virtual void addForceAndMoment( const Vector3 &r_ac_bas,
                                    const Vector3 &vel_air_bas,
                                    const Vector3 &omg_air_bas,
                                    double airDensity );

    /**
     * @brief Computes drag coefficient.
     * @param angleOfAttack [rad] angle of attack
     * @return [-] drag coefficient
     */
    virtual double getCx( double angleOfAttack ) const;

    /**
     * @brief Computes sideforce coefficient.
     * @param sideslipAngle [rad] angle of sideslip
     * @return [-] sideforce coefficient
     */
    virtual double getCy( double sideslipAngle ) const;

    /**
     * @brief Computes lift coefficient.
     * @param angleOfAttack [rad] angle of attack
     * @return [-] lift coefficient
     */
    virtual double getCz( double angleOfAttack ) const;

    /**
     * @brief Computes rolling moment coefficient.
     * @param sideslipAngle [rad] angle of sideslip
     * @return [-] rolling moment coefficient
     */
    virtual double getCl( double sideslipAngle ) const;

    /**
     * @brief Computes pitching moment coefficient.
     * @param angleOfAttack [rad] angle of attack
     * @return [-] pitching moment coefficient
     */
    virtual double getCm( double angleOfAttack ) const;

    /**
     * @brief Computes yawing moment coefficient.
     * @param sideslipAngle [rad] angle of sideslip
     * @return [-] yawing moment coefficient
     */
    virtual double getCn( double sideslipAngle ) const;
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_AERO_TAILOFF_H_
