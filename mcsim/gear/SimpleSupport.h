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
#ifndef MCSIM_GEAR_SIMPLESUPPORT_H_
#define MCSIM_GEAR_SIMPLESUPPORT_H_

////////////////////////////////////////////////////////////////////////////////

#include <mcutils/math/Vector3.h>

#include <mcsim/defs.h>
#include <mcsim/gear/BrakeGroup.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Landing gear simple support (wheel) model class.
 *
 * <h3>Refernces:</h3>
 * <ul>
 *   <li>O’Rourke J. Computational Geometry in C, 1998</li>
 *   <li>Studzinski K. Samochod. Teoria, konstrukcja i obliczenia, 1980 [in Polish]</li>
 *   <li><a href="https://apps.dtic.mil/sti/citations/ADA041920">A Solid Friction Model, ADA041920</a></li>
 *   <li>van Geffen V. A study of friction models and friction compensation, 2009</li>
 * </ul>
 */
class MCSIMAPI SimpleSupport
{
public:

    struct Data
    {
        Vector3 r_a_bas;                ///< [m] strut attachment point coordinates expressed in BAS
        Vector3 r_u_bas;                ///< [m] unloaded wheel coordinates expressed in BAS

        double k { 0.0 };               ///< [N/m] strut stiffness (linear spring) coefficient
        double c { 0.0 };               ///< [N/(m/s)] strut damping coefficient

        double mu_s { 0.0 };            ///< [-] coefficient of static friction
        double mu_k { 0.0 };            ///< [-] coefficient of kinetic friction
        double mu_r { 0.0 };            ///< [-] coefficient of rolling friction

        double delta_max { 0.0 };       ///< [rad] max turn angle

        double d_max { 0.0 };           ///< [m]   maximum distance for static friction spring like model
        double v_max { 0.0 };           ///< [m/s] maximum velocity for continuous friction model

        bool steerable { false };       ///< specifies if wheel is steerable
        bool caster    { false };       ///< specifies if wheel is caster

        BrakeGroup brakeGroup { BrakeGroup::None }; ///< brake group
    };

    /**
     * @brief Constructor.
     * @param staticFriction specifies if static friction model is enabled
     */
    SimpleSupport( bool staticFriction = true );

    /** @brief Destructor. */
    virtual ~SimpleSupport() = default;

    /**
     * @brief Computes force and moment.
     * @param vel_bas [m/s] aircraft linear velocity expressed in BAS
     * @param omg_bas [rad/s] aircraft angular velocity expressed in BAS
     * @param r_c_bas [m] contact point coordinates expressed in BAS
     * @param n_c_bas [-] contact point normal vector expressed in BAS
     * @param steering
     * @param antiskid
     * @param surf_coef
     */
    virtual void computeForceAndMoment( const Vector3 &vel_bas,
                                        const Vector3 &omg_bas,
                                        const Vector3 &r_c_bas,
                                        const Vector3 &n_c_bas,
                                        bool steering, bool antiskid,
                                        double surf_coef = 1.0 );

    /**
     * @brief Integrates wheel model.
     * @param timeStep [s] time step
     * @param vel_bas
     * @param omg_bas
     * @param r_c_bas
     * @param n_c_bas
     * @param steering
     */
    virtual void integrate( double timeStep,
                            const Vector3 &vel_bas,
                            const Vector3 &omg_bas,
                            const Vector3 &r_c_bas,
                            const Vector3 &n_c_bas,
                            bool steering );

    /**
     * @brief Update wheel model.
     * @param position [-] normalized position (0.0 - retracted, 1.0 - extended)
     * @param delta [rad] wheel steering angle
     * @param brake [-] normalized brake force
     */
    virtual void update( double position, double delta, double brake );

    inline const Vector3& getForce_BAS  () const { return _for_bas; }
    inline const Vector3& getMoment_BAS () const { return _mom_bas; }

    inline Vector3 getRa_BAS() const { return _data.r_a_bas; }
    inline Vector3 getRu_BAS() const { return _data.r_u_bas; }

    inline BrakeGroup getBrakeGroup() const { return _data.brakeGroup; }

    inline double getPosition() const { return _position; }

protected:

    Data _data;                     ///<

    Vector3 _for_bas;               ///< [N] total force vector expressed in BAS
    Vector3 _mom_bas;               ///< [N*m] total moment vector expressed in BAS

    Vector3 _r_0_wgs;               ///< [m] coordinates of the reference point for calculating static frinction expressed in WGS

    double _d_roll { 0.0 };         ///< [m] roll direction distance for static friction spring like model
    double _d_slip { 0.0 };         ///< [m] slip direction distance for static friction spring like model

    double _position { 0.0 };       ///< <0.0;1.0> normalized position (0.0 - retracted, 1.0 - extended)
    double _delta    { 0.0 };       ///< [rad] wheel turn angle
    double _brake    { 0.0 };       ///< <0.0;1.0> normalized brake force

    bool _staticFriction { true };  ///< specifies if static friction model is enabled

    /**
     * @brief Calculates wheel variables.
     * @param vel_bas [m/s]
     * @param omg_bas [rad/s]
     * @param r_c_bas [m]
     * @param n_c_bas [-]
     * @param steering
     * @param dir_lon_bas
     * @param dir_lat_bas
     * @param cosDelta
     * @param sinDelta
     * @param v_norm
     * @param v_roll
     * @param v_slip
     */
    void calculateVariables( const Vector3 &vel_bas,
                             const Vector3 &omg_bas,
                             const Vector3 &r_c_bas,
                             const Vector3 &n_c_bas,
                             bool steering,
                             Vector3 *dir_lon_bas,
                             Vector3 *dir_lat_bas,
                             double *cosDelta,
                             double *sinDelta,
                             double *v_norm,
                             double *v_roll,
                             double *v_slip );
};

} // namespace mc

////////////////////////////////////////////////////////////////////////////////

#endif // MCSIM_GEAR_SIMPLESUPPORT_H_
