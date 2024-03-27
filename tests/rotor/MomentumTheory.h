#ifndef LIBMCSIM_TESTS_ROTOR_MOMENTUMTHEORY_H_
#define LIBMCSIM_TESTS_ROTOR_MOMENTUMTHEORY_H_

/**
 * Tests auxiliary class. Uses momentum theory to calculate rotor state.
 */
class MomentumTheory
{
public:

    void Update(double climbRate, double vi0, double rho);

    inline double lambda_i() const { return _lambda_i; }
    inline double vi_vi0()   const { return _vi_vi0; }
    inline double thrust()   const { return _thrust; }

    void set_omega(double omega);
    void set_radius(double radius);

private:

    double _omega  = 0.0;   ///< [rad/s] rotor rotatational speed
    double _radius = 0.0;   ///< [m] rotor radius

    double _omegaR = 0.0;   ///< [m/s] rotor blade tip velocity
    double _area   = 0.0;   ///< [m^2] rotor disc area

    double _lambda_i = 0.0; ///< [-] normalized rotor induced velocity
    double _vi_vi0   = 0.0; ///< [-] ratio of climb rate to induced velocity at hover
    double _thrust   = 0.0; ///< [N] rotor thrust

    void UpdateDerivedVariables();
};

#endif // LIBMCSIM_TESTS_ROTOR_MOMENTUMTHEORY_H_