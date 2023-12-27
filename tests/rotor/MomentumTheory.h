#ifndef LIBMCSIM_TESTS_ROTOR_MOMENTUMTHEORY_H_
#define LIBMCSIM_TESTS_ROTOR_MOMENTUMTHEORY_H_

////////////////////////////////////////////////////////////////////////////////

/**
 * Tests auxiliary class. Uses momentum theory to calculate rotor state.
 */
class MomentumTheory
{
public:

    void Update(double climbRate, double vi0, double rho);

    inline double lambda_i() const { return lambda_i_; }
    inline double thrust()   const { return thrust_; }

    void set_omega(double omega);
    void set_radius(double radius);

private:

    double omega_  = 0.0;   ///< [rad/s] rotor rotatational speed
    double radius_ = 0.0;   ///< [m] rotor radius

    double omegaR_ = 0.0;   ///< [m/s] rotor blade tip velocity
    double area_   = 0.0;   ///< [m^2] rotor disc area

    double lambda_i_ = 0.0; ///< [-] normalized rotor induced velocity
    double thrust_   = 0.0; ///< [N] rotor thrust

    void UpdateDerivedVariables();
};

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_TESTS_ROTOR_MOMENTUMTHEORY_H_