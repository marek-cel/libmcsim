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
#ifndef MCSIM_PROP_PROPELLER_H_
#define MCSIM_PROP_PROPELLER_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Table.h>
#include <mcutils/math/Table2.h>
#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Propeller class.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>Allerton D. Principles of Flight Simulation, 2009, p.131</li>
 *   <li>Raymer D. Aircraft Design: A Conceptual Approach, 1992, p.327</li>
 *   <li>Torenbeek E. Synthesis of Subsonic Airplane Design, 1982, p.191</li>
 *   <li>Paturski Z. Przewodnik po projektach z Mechaniki Lotu, Projekt nr 5: Charakterystyki zespolu napedowego. [in Polish]</li>
 * </ul>
 */
class MCSIMAPI Propeller
{
public:

    /** Propeller direction. */
    enum Direction
    {
        CW  = 0,    ///< clockwise (looking from cockpit)
        CCW = 1     ///< counter-clockwise (looking from cockpit)
    };

    struct Data
    {
        Vector3 pos_bas;            ///< [m] propeller position expressed in BAS

        Table propPitch;            ///< [rad] propeller pitch vs [-] normalized pitch

        Table2 coefThrust;          ///< [-] thrust coefficient
        Table2 coefPower;           ///< [-] power coefficient

        Direction direction { CW }; ///< propeller direction looking from cockpit

        double gearRatio { 0.0 };   ///< [-] gear ratio (propeller rpm / engine rpm)
        double diameter  { 0.0 };   ///< [m] diameter
        double inertia   { 0.0 };   ///< [kg*m^2] polar moment of inertia
    };

    /** @brief Constructor. */
    Propeller() = default;

    /** @brief Destructor. */
    virtual ~Propeller() = default;

    /**
     * @brief Computes thrust.
     * @param airspeed [m/s] airspeed
     * @param airDensity [kg/m^3] air density
     */
    virtual void computeThrust( double airspeed, double airDensity );

    /**
     * @brief Integrates model.
     * @param dt [s] time step
     * @param engineInertia [kg*m^2] engine polar moment of inertia
     */
    virtual void integrate( double dt, double engineInertia );

    /**
     * @brief Updates propeller.
     * @param normPitch    <0.0;1.0> normalized propeller lever position
     * @param engineTorque [N]       engine torque
     * @param airspeed     [m/s]     airspeed
     * @param airDensity   [kg/m^3]  air density
     */
    virtual void update( double propellerLever,
                         double engineTorque,
                         double airspeed,
                         double airDensity );

    /**
     * @brief Returns propeller direction.
     * @return propeller direction
     */
    inline Direction getDirection() const
    {
        return _data.direction;
    }

    /**
     * @brief Returns engine rpm.
     * @return [rpm] engine rpm
     */
    inline double getEngineRPM() const
    {
        return _speed_rpm / _data.gearRatio;
    }

    /**
     * @brief Returns propeller rpm.
     * @return [rpm] propeller rpm
     */
    inline double getRPM() const
    {
        return _speed_rpm;
    }

    /**
     * @brief Returns propeller polar moment of inertia.
     * @return [kg*m^2] propeller polar moment of inertia
     */
    inline double getInertia() const
    {
        return _data.inertia;
    }

    /**
     * @brief Returns propeller angular velocity.
     * @return [rad/s] propeller angular velocity
     */
    inline double getOmega() const
    {
        return _omega;
    }

    /**
     * @brief Returns propeller position expressed in BAS.
     * @return [m] propeller position expressed in BAS
     */
    inline const Vector3& getPos_BAS() const
    {
        return _data.pos_bas;
    }

    /**
     * @brief Returns propeller thrust.
     * @return [N] propeller thrust
     */
    inline double getThrust() const
    {
        return _thrust;
    }

    /**
     * @brief Returns induced velocity.
     * @return [m/s] induced velocity
     */
    inline double getInducedVelocity() const
    {
        return _inducedVelocity;
    }

    /**
     * @brief Returns torque.
     * @return [N*m] torque
     */
    inline double getTorque() const
    {
        return ( _torqueRequired < _torqueAvailable ) ? _torqueRequired : _torqueAvailable;
    }

    void setRPM( double rpm );

protected:

    Data _data;                         ///<

    double _area { 0.0 };               ///< [m^2] propeller disc area

    double _pitch     { 0.0 };          ///< [rad]   propeller pitch at 0.75 radius
    double _omega     { 0.0 };          ///< [rad/s] propeller angular velocity
    double _speed_rps { 0.0 };          ///< [rps]   propeller speed
    double _speed_rpm { 0.0 };          ///< [rpm]   propeller speed
    double _thrust    { 0.0 };          ///< [N] t   hrust

    double _inducedVelocity { 0.0 };    ///< [m/s] induced velocity

    double _torqueAvailable { 0.0 };    ///< [N*m] available torque
    double _torqueRequired  { 0.0 };    ///< [N*m] required torque

    /**
     * @brief Computes induced velocity.
     * @param airspeed [m/s] airspeed
     * @param airDensity [kg/m^3] air density
     */
    virtual double getInducedVelocity( double airspeed, double airDensity );

    /**
     * @brief Computes propeller pitch.
     * @param propellerLever <0.0;1.0> normalized propeller lever position
     */
    virtual double getPropellerPitch( double propellerLever );
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_PROP_PROPELLER_H_
