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
#ifndef LIBMCSIM_AIRCRAFT_H
#define LIBMCSIM_AIRCRAFT_H

////////////////////////////////////////////////////////////////////////////////

#include <memory>
#include <vector>

#include <mcutil/geo/ECEF.h>

#include <mcutil/math/Angles.h>
#include <mcutil/math/Matrix3x3.h>
#include <mcutil/math/Quaternion.h>
#include <mcutil/math/Vector3.h>

#include <mcutil/math/IIntegrator.h>

#include <mcutil/xml/XmlNode.h>

#include <mcsim/defs.h>

#include <mcsim/IAircraft.h>
#include <mcsim/IBuilder.h>

#include <mcsim/IEnvironment.h>
#include <mcsim/IIntersections.h>

#include <mcsim/IControls.h>

#include <mcsim/IAerodynamics.h>
#include <mcsim/ILandingGear.h>
#include <mcsim/IMass.h>
#include <mcsim/IPropulsion.h>

#include <mcsim/IRecorder.h>

#include <mcsim/CrashCause.h>
#include <mcsim/StateIndex.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

/**
 * @brief Aircraft model base class.
 *
 * Aircraft flight dynamics model base class.
 *
 * @section Conventions and Units
 *
 * Units:
 * Flight Dynamics Model uses International System of Units (SI) for all
 * internal computations.
 * Other units can be used in XML data files.
 * Make sure to use "unit", "keys_unit", "cols_unit" or "rows_unit"
 * attribute when using non SI units in XML data file.
 *
 * @see Units::getConverter()
 *
 * @subsection Rotations
 *
 * Rotations angles are expressed as Bryant angles (Euler angles in z-y-x
 * convention).
 * All rotations and rotation related operations are considered to be
 * passive (alias) rotations.
 *
 * @see https://en.wikipedia.org/wiki/Active_and_passive_transformation
 *
 * @subsection Coordinate Systems
 *
 * All coordinate systems are right-handed.
 *
 * Body Axis System (BAS)
 * Body Axis System is the body-fixed coordinate system, with the x-axis
 * positive forwards, the y-axis positive right and z-axis positive
 * downwards.
 *
 * North-East-Down (NED)
 * Local ground axis system with x-axis positive North, y-axis positive East
 * and z-axis positive downwards.
 *
 * East-North-Up (ENU)
 * Local ground axis system with x-axis positive East, y-axis positive North
 * and z-axis positive upwards.
 *
 * Global Coordinate System (GCS)
 * Global Coordinate System is a right-handed coordinate system in which body
 * position and attitude is defined. In most cases local ground axis system or
 * Earth-centered, Earth-fixed (ECEF), such as World Geodetic System 1984 should
 * be used.
 *
 * @subsection XML configuration file format
 *
 * @code
 * <sim>
 *   <collision_points>
 *     <collision_point> { [m] x-coordinate } { [m] y-coordinate } { [m] z-coordinate } </collision_point>
 *     ... { more entries }
 *   </collision_points>
 *   <limitations>
 *     <airspeed_max> { [m/s] maximum airspeed (exceeding this value causes crash) } </airspeed_max>
 *     <load_aero_min> { [-] minimum (maximum negative) load factor due to aerodynamics (exceeding this value causes crash) } </load_aero_min>
 *     <load_aero_max> { [-] maximum (maximum positive) load factor due to aerodynamics (exceeding this value causes crash) } </load_aero_max>
 *     <load_gear_max> { [-] maximum absolute load factor due to landing gear (exceeding this value causes crash) } </load_gear_max>
 *   </limitations>
 *   <pilot_position> { [m] x-coordinate } { [m] y-coordinate } { [m] z-coordinate } </pilot_position>
 *   <aerodynamics>
 *     { aerodynamics data }
 *   </aerodynamics>
 *   <controls>
 *     { controls data }
 *   </controls>
 *   <landing_gear>
 *     { landing gear data }
 *   </landing_gear>
 *   <mass>
 *     { mass data }
 *   </mass>
 *   <propulsion>
 *     { propulsion data }
 *   </propulsion>
 * </sim>
 * @endcode
 *
 * @see Cel M.: Flight Dynamics Model for the Real-Time Flight Simulation, 2020
 * @see Taylor J.: Classical Mechanics, 2005
 * @see Osinski Z.: Mechanika ogolna, 1997, [in Polish]
 * @see Allerton D.: Principles of Flight Simulation, 2009
 * @see Stevens B., Lewis F.: Aircraft Control and Simulation, 1992
 * @see Maryniak J.: Ogolny model matematyczny sterowanego samolotu, 1992 [in Polish]
 * @see Sibilski K.: Modelowanie i symulacja dynamiki ruchu obiektow latajacych, 2004 [in Polish]
 * @see Narkiewicz J.: Tiltrotor Modelling for Simulation in Various Flight Conditions, 2006
 * @see Zlocka M.: Wyklady z dynamiki lotu, 2008 [in Polish]
 * @see https://en.wikipedia.org/wiki/Rotating_reference_frame
 * @see https://en.wikipedia.org/wiki/Centrifugal_force#Derivation
 */
class MCSIMEXPORT Aircraft : public IAircraft
{
public:

    using CollisionPoints = std::vector< Vector3 >; ///< collision points
    using StateVector     = VectorN;

    /** @brief Constructor. */
    Aircraft( IBuilderPtrS builder );

    /**
     * @brief Reads data.
     * @param dataNode XML node
     */
    virtual void readData( XmlNode &dataNode );

    /**
     * @brief Initializes aircraft.
     * @param engineOn specifies if engine is running on startup
     */
    virtual void initialize( bool initEngineOn = false );

    /**
     * @brief Updates aircraft due to simulation time step.
     * @param timeStep simulation time step [s]
     * @param integrate specifies integration is enabled
     */
    virtual void update( double timeStep, bool integrate = true );

    inline const StateVector& getStateVect() const { return _stateVect; }
    inline const StateVector& getDerivVect() const { return _derivVect; }

    inline double getTimeStep() const override { return _timeStep; }

    inline const Vector3&    getPos_GCS() const override { return _pos_gcs; }
    inline const Quaternion& getAtt_GCS() const override { return _att_gcs; }
    inline const Vector3&    getVel_BAS() const override { return _vel_bas; }
    inline const Vector3&    getOmg_BAS() const override { return _omg_bas; }

    inline const ECEF& getGCS() const override { return _gcs; }

    inline const Matrix3x3& getGCS2BAS() const override { return _gcs2bas; }
    inline const Matrix3x3& getBAS2GCS() const override { return _bas2gcs; }
    inline const Matrix3x3& getGCS2NED() const override { return _gcs2ned; }
    inline const Matrix3x3& getNED2GCS() const override { return _ned2gcs; }

    inline const Matrix3x3& getNED2BAS() const override { return _ned2bas; }
    inline const Matrix3x3& getBAS2NED() const override { return _bas2ned; }

    inline const Angles& getAngles_GCS() const override { return _angles_gcs; }
    inline const Angles& getAngles_NED() const override { return _angles_ned; }

    inline const Vector3& getVel_NED() const override { return _vel_ned; }

    inline const Vector3& getVel_air_BAS() const override { return _vel_air_bas; }
    inline const Vector3& getOmg_air_BAS() const override { return _omg_air_bas; }

    inline const Vector3& getAcc_BAS() const override { return _acc_bas; }
    inline const Vector3& getEps_BAS() const override { return _eps_bas; }

    inline const Vector3& getGrav_GCS() const override { return _grav_gcs; }
    inline const Vector3& getGrav_BAS() const override { return _grav_bas; }

    inline const Vector3& getGForceBAS() const override { return _g_force_bas; }
    inline const Vector3& getGPilotBAS() const override { return _g_pilot_bas; }

    inline const Vector3& getGround_GCS() const override { return _ground_gcs; }
    inline const Vector3& getGround_BAS() const override { return _ground_bas; }

    inline const Vector3& getNormal_GCS() const override { return _normal_gcs; }
    inline const Vector3& getNormal_BAS() const override { return _normal_bas; }

    inline double getElevation     () const override { return _elevation;     }
    inline double getAltitude_ASL  () const override { return _altitude_asl;  }
    inline double getAltitude_AGL  () const override { return _altitude_agl;  }
    inline double getRoll          () const override { return _roll;          }
    inline double getPitch         () const override { return _pitch;         }
    inline double getHeading       () const override { return _heading;       }
    inline double getAngleOfAttack () const override { return _angleOfAttack; }
    inline double getSideslipAngle () const override { return _sideslipAngle; }
    inline double getClimbAngle    () const override { return _climbAngle;    }
    inline double getTrackAngle    () const override { return _trackAngle;    }
    inline double getSlipSkidAngle () const override { return _slipSkidAngle; }
    inline double getAirspeed      () const override { return _airspeed;      }
    inline double getIAS           () const override { return _ias;           }
    inline double getTAS           () const override { return _tas;           }
    inline double getGroundSpeed   () const override { return _groundSpeed;   }
    inline double getDynPress      () const override { return _dynPress;      }
    inline double getMachNumber    () const override { return _machNumber;    }
    inline double getClimbRate     () const override { return _climbRate;     }
    inline double getTurnRate      () const override { return _turnRate;      }

    inline double getDistanceTraveled() const override { return _distanceTraveled; }

    /**
     * @brief Sets aircraft state vector.
     * This function is meant to set initial conditions at the beginning,
     * as well as to reposition aircraft during flight.
     * @param state state vector
     */
    virtual void setStateVector( const StateVector &stateVector );

    inline void setFreezePosition( bool freeze_position ) { _freeze_position = freeze_position; }
    inline void setFreezeAttitude( bool freeze_attitude ) { _freeze_attitude = freeze_attitude; }
    inline void setFreezeVelocity( bool freeze_velocity ) { _freeze_velocity = freeze_velocity; }

protected:

    IEnvironmentPtrS   _envir;          ///< environment model
    IIntersectionsPtrS _isect;          ///< intersections calculator

    IControlsPtrS      _ctrl;           ///< flight controls model

    IAerodynamicsPtrS  _aero;           ///< aerodynamics model
    ILandingGearPtrS   _gear;           ///< landing gear model
    IMassPtrS          _mass;           ///< mass and inertia model
    IPropulsionPtrS    _prop;           ///< propulsion model

    IRecorderPtrS _recorder;            ///< recording/replaying module

    StateVector _stateVect;             ///< aircraft state vector
    StateVector _statePrev;             ///< aircraft state vector (previous)
    StateVector _derivVect;             ///< aircraft state vector derivative (for output purposes only)

    IIntegratorPtrS _integrator;        ///< integration procedure object

    CollisionPoints _collisionPoints;   ///< [m] collision points expressed in BAS

    double _airspeed_max;               ///< [m/s] maximum indicated airspeed (exceeding this value causes crash)
    double _load_aero_min;              ///< [-] minimum (maximum negative) load factor due to aerodynamics (exceeding this value causes crash)
    double _load_aero_max;              ///< [-] maximum (maximum positive) load factor due to aerodynamics (exceeding this value causes crash)
    double _load_gear_max;              ///< [-] maximum absolute load factor due to landing gear (exceeding this value causes crash)

    Vector3 _pos_pilot_bas;             ///< [m] pilot's head position expressed in BAS

    double _timeStep;                   ///< [s] simulation time step

    Vector3    _pos_gcs;                ///< [m] aircraft position expressed in GCS
    Quaternion _att_gcs;                ///< aircraft attitude expressed as quaternion of rotation from GCS to BAS
    Vector3    _vel_bas;                ///< [m/s] aircraft linear velocity vector expressed in BAS
    Vector3    _omg_bas;                ///< [rad/s] aircraft angular velocity expressed in BAS

    ECEF _gcs;                          ///< aircraft ECEF position

    Matrix3x3 _gcs2bas;                 ///< matrix of rotation from GCS to BAS
    Matrix3x3 _bas2gcs;                 ///< matrix of rotation from BAS to GCS
    Matrix3x3 _gcs2ned;                 ///< matrix of rotation from GCS to NED
    Matrix3x3 _ned2gcs;                 ///< matrix of rotation from NED to GCS

    Matrix3x3 _ned2bas;                 ///< matrix of rotation from NED to BAS
    Matrix3x3 _bas2ned;                 ///< matrix of rotation from BAS to NED

    Angles _angles_gcs;                 ///< [rad] aircraft attitude expressed as rotation from GCS to BAS
    Angles _angles_ned;                 ///< [rad] aircraft attitude expressed as rotation from NED to BAS

    Vector3 _vel_ned;                   ///< [m/s] aircraft linear velocity vector expressed in NED

    Vector3 _vel_air_bas;               ///< [m/s] aircraft linear velocity vector relative to the air expressed in BAS
    Vector3 _omg_air_bas;               ///< [rad/s] aircraft angular velocity relative to the air expressed in BAS

    Vector3 _acc_bas;                   ///< [m/s^2] aircraft linear acceleration vector expressed in BAS
    Vector3 _eps_bas;                   ///< [rad/s^2] aircraft angular acceleration vector expressed in BAS

    Vector3 _grav_gcs;                  ///< [m/s^2] gravity acceleration vector expressed in GCS
    Vector3 _grav_bas;                  ///< [m/s^2] gravity acceleration vector expressed in BAS

    Vector3 _g_force_bas;               ///< [-] vector of G-Force factor (aircraft) expressed in BAS
    Vector3 _g_pilot_bas;               ///< [-] vector of G-Force factor (pilot's head) expressed in BAS

    Vector3 _ground_gcs;                ///< [m] ground intersection coordinates expressed in GCS
    Vector3 _ground_bas;                ///< [m] ground intersection coordinates expressed in BAS

    Vector3 _normal_gcs;                ///< [-] normal to ground vector expressed in GCS
    Vector3 _normal_bas;                ///< [-] normal to ground vector expressed in BAS

    CrashCause _crashCause;             ///< crash cause

    uint32_t _collisionPointIndex;      ///< current collision point index

    double _elevation;                  ///< [m] ground elevation above mean sea level
    double _altitude_asl;               ///< [m] altitude above sea level
    double _altitude_agl;               ///< [m] altitude above ground level
    double _roll;                       ///< [rad] roll angle
    double _pitch;                      ///< [rad] pitch angle
    double _heading;                    ///< [rad] true heading
    double _angleOfAttack;              ///< [rad] angle of attack
    double _sideslipAngle;              ///< [rad] sideslip angle
    double _climbAngle;                 ///< [rad] climb angle
    double _trackAngle;                 ///< [rad] track angle
    double _slipSkidAngle;              ///< [rad] slip/skid angle
    double _airspeed;                   ///< [m/s] real airspeed
    double _dynPress;                   ///< [Pa] dynamic pressure
    double _ias;                        ///< [m/s] indicated airspeed
    double _tas;                        ///< [m/s] true airspeed
    double _groundSpeed;                ///< [m/s] ground speed (horizontal velocity)
    double _machNumber;                 ///< [-] Mach number
    double _climbRate;                  ///< [m/s] climb rate
    double _turnRate;                   ///< [rad/s] turn rate
    double _headingPrev;                ///< [rad] previous heading

    double _distanceTraveled;           ///< [m] distance traveled

    bool _freeze_position;              ///< specifies if position is to be frozen (is not integrating)
    bool _freeze_attitude;              ///< specifies if attitude is to be frozen (is not integrating)
    bool _freeze_velocity;              ///< specifies if velocity is to be frozen (is not integrating)

    /** @brief This function is called just before time integration step. */
    virtual void anteIntegration();

    /** @brief This function is called just after time integration step. */
    virtual void postIntegration();

    /** @brief This function checks collisions. */
    virtual void detectCrash();

    /**
     * @brief Computes state vector derivatives due to given state vector.
     * @param stateVect state vector
     * @param derivVect resulting state vector derivative
     */
    virtual void computeStateDeriv( const StateVector &stateVect,
                                    StateVector *derivVect );

    /**
     * @brief Updates aircraft state variables.
     * @param stateVect state vector
     * @param derivVect state vector derivative
     */
    virtual void updateVariables( const StateVector &stateVect,
                                  const StateVector &derivVect );

private:

    /** Using this constructor is forbidden. */
    Aircraft( const Aircraft& ) = delete;

    /** Using this constructor is forbidden. */
    Aircraft( const Aircraft&& ) = delete;
};

} // mc

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_AIRCRAFT_H
