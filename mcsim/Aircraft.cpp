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

#include <mcsim/Aircraft.h>

#include <algorithm>
#include <functional>

#include <mcutil/math/GaussJordan.h>
#include <mcutil/math/Math.h>
#include <mcutil/physics/Physics.h>

#include <mcsim/aerodynamics/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

namespace mc
{

////////////////////////////////////////////////////////////////////////////////

Aircraft::Aircraft( IBuilderPtrS builder )
    : _stateVect ( std::max( 13, builder->getStateDimension() ) )
    , _statePrev ( std::max( 13, builder->getStateDimension() ) )
    , _derivVect ( std::max( 13, builder->getStateDimension() ) )

    , _airspeed_max  ( 0.0 )
    , _load_aero_min ( 0.0 )
    , _load_aero_max ( 0.0 )
    , _load_gear_max ( 0.0 )

    , _timeStep ( 0.0 )

    , _gcs ( builder->getECEF() )

    , _crashCause ( CrashCause::NoCrash )

    , _collisionPointIndex ( 0 )

    , _elevation     ( 0.0 )
    , _altitude_asl  ( 0.0 )
    , _altitude_agl  ( 0.0 )
    , _roll          ( 0.0 )
    , _pitch         ( 0.0 )
    , _heading       ( 0.0 )
    , _angleOfAttack ( 0.0 )
    , _sideslipAngle ( 0.0 )
    , _climbAngle    ( 0.0 )
    , _trackAngle    ( 0.0 )
    , _slipSkidAngle ( 0.0 )
    , _airspeed      ( 0.0 )
    , _dynPress      ( 0.0 )
    , _ias           ( 0.0 )
    , _tas           ( 0.0 )
    , _groundSpeed   ( 0.0 )
    , _machNumber    ( 0.0 )
    , _climbRate     ( 0.0 )
    , _turnRate      ( 0.0 )
    , _headingPrev   ( 0.0 )

    , _distanceTraveled ( 0.0 )

    , _freeze_position ( false )
    , _freeze_attitude ( false )
    , _freeze_velocity ( false )
{
    builder->setAircraft( IAircraftPtrS(this) );

    _envir = builder->getEnvironment();
    _isect = builder->getIntersections();

    _ctrl  = builder->getControls();

    _aero  = builder->getAerodynamics ();
    _gear  = builder->getLandingGear  ();
    _mass  = builder->getMass         ();
    _prop  = builder->getPropulsion   ();

    _recorder = builder->getRecorder();

    _integrator = builder->getIntegrator();

    _integrator->setDerivFun(
        [this]( const StateVector &stateVect, StateVector *derivVect )
        {
            computeStateDeriv( stateVect, derivVect );
        }
    );
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::readData( XmlNode &dataNode )
{
    if ( dataNode.isValid() )
    {

    }
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::initialize( bool initEngineOn )
{
    _ctrl->initialize();

    _aero->initialize();
    _gear->initialize();
    _mass->initialize();
    _prop->initialize( initEngineOn );

    updateVariables( _stateVect, _derivVect );
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::update( double timeStep, bool integrate )
{
    _timeStep = timeStep;

    anteIntegration();

    if ( integrate )
    {
        /////////////////////////////////////////////////
        _integrator->integrate( _timeStep, &_stateVect );
        /////////////////////////////////////////////////

        _ctrl->integrate( _timeStep );

        _aero->integrate( _timeStep );
        _gear->integrate( _timeStep );
        _mass->integrate( _timeStep );
        _prop->integrate( _timeStep );

        _distanceTraveled += _timeStep * _vel_ned.getLengthXY();
    }

    postIntegration();
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::setStateVector( const StateVector &stateVector )
{
    _stateVect = stateVector;
    _statePrev = _stateVect;

    anteIntegration();
    computeStateDeriv( _stateVect, &_derivVect );
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::anteIntegration()
{
    updateVariables( _stateVect, _derivVect );

    _envir->update();
    _isect->update();

    _ctrl->update();

    _aero->update();
    _gear->update();
    _mass->update();
    _prop->update();
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::postIntegration()
{
    _att_gcs.normalize();

    _stateVect( StateIndex::e0 ) = _att_gcs.e0();
    _stateVect( StateIndex::ex ) = _att_gcs.ex();
    _stateVect( StateIndex::ey ) = _att_gcs.ey();
    _stateVect( StateIndex::ez ) = _att_gcs.ez();

    if ( _stateVect.isValid() )
    {
        if ( _timeStep > 1.0e-12 )
        {
            _derivVect = ( _stateVect - _statePrev   ) / _timeStep;
            _turnRate  = ( _heading   - _headingPrev ) / _timeStep;
        }

        _statePrev = _stateVect;
        _headingPrev = _heading;

        updateVariables( _stateVect, _derivVect );
    }

    detectCrash();
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::detectCrash()
{
    // detect collisions
    if ( _crashCause == CrashCause::NoCrash )
    {
        if ( _isect->isIntersection( _pos_gcs, _pos_gcs
                + _bas2gcs * _collisionPoints[_collisionPointIndex] ) )
        {
            _crashCause = CrashCause::Collision;
        }

        _collisionPointIndex++;

        if ( !( _collisionPointIndex < _collisionPoints.size() ) )
            _collisionPointIndex = 0;
    }

    // detect overspeed
    if ( _crashCause == CrashCause::NoCrash )
    {
        if ( _ias > _airspeed_max )
        {
            _crashCause = CrashCause::Overspeed;
        }
    }

    // detect overload
    if ( _crashCause == CrashCause::NoCrash )
    {
        double weight_inv = 1.0 / ( _mass->getMass() * Physics::standardGravity );

        double load_factor_aero = _aero->getForce_BAS().getLength() * weight_inv;
        double load_factor_gear = _gear->getForce_BAS().getLength() * weight_inv;

        if ( load_factor_aero > _load_aero_max
          || load_factor_aero < _load_aero_min
          || load_factor_gear > _load_gear_max )
        {
            _crashCause = CrashCause::Overstress;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::computeStateDeriv( const StateVector &stateVect,
                                  StateVector *derivVect )
{
    updateVariables( stateVect, *derivVect );

    // computing forces and moments
    _aero->computeForceAndMoment();
    _gear->computeForceAndMoment();
    _mass->computeForceAndMoment();
    _prop->computeForceAndMoment();

    Vector3 for_bas = _aero->getForce_BAS()
                    + _mass->getForce_BAS()
                    + _gear->getForce_BAS()
                    + _prop->getForce_BAS();

    Vector3 mom_bas = _aero->getMoment_BAS()
                    + _mass->getMoment_BAS()
                    + _gear->getMoment_BAS()
                    + _prop->getMoment_BAS();

    // computing position derivatives
    Vector3 pos_dot_gcs = _bas2gcs * _vel_bas;

    if ( _freeze_position )
    {
        (*derivVect)( StateIndex::x ) = 0.0;
        (*derivVect)( StateIndex::y ) = 0.0;
        (*derivVect)( StateIndex::z ) = 0.0;
    }
    else
    {
        (*derivVect)( StateIndex::x ) = pos_dot_gcs.x();
        (*derivVect)( StateIndex::y ) = pos_dot_gcs.y();
        (*derivVect)( StateIndex::z ) = pos_dot_gcs.z();
    }


    // computing attitude derivatives
    Quaternion att_dot_gcs = _att_gcs.getDerivative( _omg_bas, 2.0 * _timeStep );

    if ( _freeze_attitude )
    {
        (*derivVect)( StateIndex::e0 ) = 0.0;
        (*derivVect)( StateIndex::ex ) = 0.0;
        (*derivVect)( StateIndex::ey ) = 0.0;
        (*derivVect)( StateIndex::ez ) = 0.0;
    }
    else
    {
        (*derivVect)( StateIndex::e0 ) = att_dot_gcs.e0();
        (*derivVect)( StateIndex::ex ) = att_dot_gcs.ex();
        (*derivVect)( StateIndex::ey ) = att_dot_gcs.ey();
        (*derivVect)( StateIndex::ez ) = att_dot_gcs.ez();
    }

    // computing linear and angular velocities derivatives
    double mass = _mass->getMass();
    Matrix3x3 it_bas = _mass->getInertiaTensor();
    Vector3 st_bas = _mass->getFirstMomentOfMass();

    // momentum and angular momentum
    Vector3 p_bas = mass   * _vel_bas + ( _omg_bas % st_bas );
    Vector3 h_bas = it_bas * _omg_bas + ( st_bas % _vel_bas );

    // right-hand-sideforce vector
    Vector3 for_rhs = for_bas - ( _omg_bas % p_bas );

    // right-hand-side moment vector
    Vector3 mom_rhs = mom_bas - ( _vel_bas % p_bas ) - ( _omg_bas % h_bas );

    // right-hand-side combined vector
    Vector6 vec_rhs;

    vec_rhs( 0 ) = for_rhs.x();
    vec_rhs( 1 ) = for_rhs.y();
    vec_rhs( 2 ) = for_rhs.z();
    vec_rhs( 3 ) = mom_rhs.x();
    vec_rhs( 4 ) = mom_rhs.y();
    vec_rhs( 5 ) = mom_rhs.z();

    // inertia matrix
    Matrix6x6 mi_bas = _mass->getInertiaMatrix();

    // state derivatives (results)
    Vector6 acc_bas;

    GaussJordan::solve<6>( mi_bas, vec_rhs, &acc_bas );

    // Coriolis effect due to Earth rotation
    Vector3 acc_coriolis_bas = -2.0 * ( _gcs2bas * ( _envir->getRotation_GCS() % _vel_bas ) );

    acc_bas( 0 ) += acc_coriolis_bas.x();
    acc_bas( 1 ) += acc_coriolis_bas.y();
    acc_bas( 2 ) += acc_coriolis_bas.z();

    if ( !_freeze_velocity )
    {
        // rewriting acceleration into state derivatives vector
        for ( int i = StateIndex::u; i < MCSIM_STATE_DIMENSION; ++i )
        {
            (*derivVect)( i ) = acc_bas( i - StateIndex::u );
        }
    }
    else
    {
        for ( int i = StateIndex::u; i < MCSIM_STATE_DIMENSION; ++i )
        {
            (*derivVect)( i ) = 0.0;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void Aircraft::updateVariables( const StateVector &stateVect,
                                const StateVector &derivVect )
{
    _pos_gcs.set( stateVect( StateIndex::x ),
                  stateVect( StateIndex::y ),
                  stateVect( StateIndex::z ) );

    _att_gcs.set( stateVect( StateIndex::e0 ),
                  stateVect( StateIndex::ex ),
                  stateVect( StateIndex::ey ),
                  stateVect( StateIndex::ez ) );

    _vel_bas.set( stateVect( StateIndex::u ),
                  stateVect( StateIndex::v ),
                  stateVect( StateIndex::w ) );

    _omg_bas.set( stateVect( StateIndex::p ),
                  stateVect( StateIndex::q ),
                  stateVect( StateIndex::r ) );

    _gcs.setPos_ECEF( _pos_gcs );

    _envir->update( _gcs );
    _isect->update( _gcs );

    _gcs2bas = Matrix3x3( _att_gcs );
    _gcs2ned = Matrix3x3( _gcs.getECEF2NED() );
    _bas2gcs = _gcs2bas.getTransposed();
    _ned2gcs = _gcs2ned.getTransposed();

    _ned2bas = Matrix3x3( _gcs.getNED2BAS( _att_gcs ) );
    _bas2ned = _ned2bas.getTransposed();

    _angles_gcs = _gcs2bas.getAngles();
    _angles_ned = _ned2bas.getAngles();

    _vel_ned = _bas2ned * _vel_bas;

    _vel_air_bas = _vel_bas - _ned2bas * _envir->getWind_NED();
    _omg_air_bas = _omg_bas;

    _acc_bas.u() = derivVect( StateIndex::u );
    _acc_bas.v() = derivVect( StateIndex::v );
    _acc_bas.w() = derivVect( StateIndex::w );

    _eps_bas.p() = derivVect( StateIndex::p );
    _eps_bas.q() = derivVect( StateIndex::q );
    _eps_bas.r() = derivVect( StateIndex::r );

    _grav_gcs = _envir->getGravity_GCS();
    _grav_bas = _gcs2bas * _grav_gcs;

    Vector3 acc_gforce_bas = _acc_bas + ( _omg_bas % _vel_bas );
    Vector3 acc_gpilot_bas = acc_gforce_bas
                           + ( _omg_bas % ( _omg_bas % _pos_pilot_bas ) )
                           + ( _eps_bas % _pos_pilot_bas );

    _g_force_bas = -( acc_gforce_bas - _grav_bas ) / Physics::standardGravity;
    _g_pilot_bas = -( acc_gpilot_bas - _grav_bas ) / Physics::standardGravity;

    Geo e_isect_geo = _gcs.getPos_Geo();
    e_isect_geo.alt = -1000.0;

    Vector3 ground_gcs;
    Vector3 normal_gcs;

    if ( _isect->getIntersection( _pos_gcs, _gcs.geo2ecef( e_isect_geo ),
                                  &ground_gcs, &normal_gcs ) )
    {
        _ground_gcs = ground_gcs;
        _normal_gcs = normal_gcs;
    }

    _ground_bas = _gcs2bas * ( _ground_gcs - _pos_gcs );
    _normal_bas = _gcs2bas * _normal_gcs;

    _elevation = _gcs.ecef2geo( _ground_gcs ).alt;

    _altitude_asl = _gcs.getPos_Geo().alt;
    _altitude_agl = _altitude_asl - _elevation;

    _roll    = _angles_ned.phi();
    _pitch   = _angles_ned.tht();
    _heading = _angles_ned.psi();

    _angleOfAttack = mc::getAngleOfAttack( _vel_air_bas );
    _sideslipAngle = mc::getSideslipAngle( _vel_air_bas );

    double vel_ne = _vel_ned.getLengthXY();

    _trackAngle = atan2( _vel_ned.v(), _vel_ned.u() );
    _climbAngle = atan2( -_vel_ned.w(), vel_ne );

    while ( _trackAngle < 0.0      ) _trackAngle += 2.0*M_PI;
    while ( _trackAngle > 2.0*M_PI ) _trackAngle -= 2.0*M_PI;

    _slipSkidAngle = atan2( -_g_pilot_bas.y(), _g_pilot_bas.z() );

    double dynPressLon = 0.5 * _envir->getDensity() * Math::pow2( _vel_air_bas.u() );

    _airspeed    = _vel_air_bas.getLength();
    _dynPress    = 0.5 * _envir->getDensity() * Math::pow2( _airspeed );
    _ias         = sqrt( 2.0 * dynPressLon / _envir->getStdRefLevelDensity() );
    _tas         = _ias * sqrt( _envir->getStdRefLevelDensity() / _envir->getDensity() );
    _groundSpeed = _vel_ned.getLengthXY();
    _machNumber  = _envir->getSpeedOfSound() > 0.0 ? ( _airspeed / _envir->getSpeedOfSound() ) : 0.0;
    _climbRate   = -_vel_ned.z();
}

////////////////////////////////////////////////////////////////////////////////

} // mc
