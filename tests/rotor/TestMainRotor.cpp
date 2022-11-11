#include <gtest/gtest.h>

#include <fstream>

#include <mcutils/misc/Units.h>

#include <mcsim/rotor/MainRotor.h>

////////////////////////////////////////////////////////////////////////////////

class TestMainRotor : public ::testing::Test
{
protected:

    class MainRotorAdapter : public mc::MainRotor
    {
    public:

        void updateFlappingAnglesThrustCoefsAndVelocity( double mu_x, double mu_x2, double mu_z,
                                                         double p, double q, double a_z,
                                                         double gamma )
        {
            mc::MainRotor::updateFlappingAnglesThrustCoefsAndVelocity( mu_x, mu_x2, mu_z,
                                                                       p, q, a_z,
                                                                       gamma );
        }
    };

    const double airDensity = 1.225;
    const double gravAcc = 9.80665;

    const double rotor_radius = 8.18;
    const double rotor_omega = 2 * M_PI *  258.0 / 60.0;
    const double rotor_omegaR = rotor_omega * rotor_radius;
    const double rotor_area = M_PI * pow( rotor_radius, 2.0 );

    const double helicopter_mass   = 6000.0;
    const double helicopter_weight = helicopter_mass * gravAcc;

    TestMainRotor() {}
    virtual ~TestMainRotor() {}

    void SetUp() override {}
    void TearDown() override {}

    mc::MainRotor::Data getMainRotorData();
};

////////////////////////////////////////////////////////////////////////////////

mc::MainRotor::Data TestMainRotor::getMainRotorData()
{
    mc::MainRotor::Data data;

    data.r_hub_bas = mc::Vector3();

    data.inclination = 0.0;

    data.ccw = true;

    data.nb = 4;

    data.blade_mass = 116.5;

    data.r = rotor_radius;
    data.c = 0.53;
    data.e = 0.38;

    data.a = 5.73;
    data.b = 0.97;

    data.delta_0 = 0.0;
    data.delta_2 = 0.0;

    data.beta_max = mc::Units::deg2rad( 20.0 );

    data.ct_max = DBL_MAX;
    data.ch_max = DBL_MAX;
    data.cq_max = DBL_MAX;

    data.thrust_factor = 1.0;
    data.hforce_factor = 1.0;
    data.torque_factor = 1.0;

    data.vrs_thrust_factor = 1.0;
    data.vrs_torque_factor = 1.0;

    return data;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, SaveResultsToFile1)
{
    std::ofstream file;
    file.open( "../main_rotor1.csv", std::ios_base::out );

    if ( file.is_open() )
    {
        file << R"##("vc")##";
        file << "\t";
        file << R"##("vi0")##";
        file << "\t";
        file << R"##("vi")##";
        file << "\t";
        file << R"##("lambda_i")##";
        file << "\t";
        file << R"##("vc_vi0")##";
        file << "\t";
        file << R"##("vi_vi0")##";
        file << "\t";
        file << R"##("thrust")##";
        file << "\t";
        file << R"##("torque")##";

        file << "\t";
        file << R"##("mt_vi")##";
        file << "\t";
        file << R"##("mt_lambda_i")##";
        file << "\t";
        file << R"##("mt_vi_vi0")##";
        file << "\t";
        file << R"##("mt_thrust")##";
        file << std::endl;

        mc::MainRotor mr;
        mr.setData( getMainRotorData() );

        double collective = mc::Units::deg2rad( 6.0 );
        mr.update( rotor_omega, 0.0, collective, 0.0, 0.0 );

        const double climbRate_min  = -30.0;
        const double climbRate_max  =  10.0;
        const double climbRate_step =   0.1;

        double climbRate = climbRate_min;

        do
        {
            // main rotor
            mc::Vector3 vel_bas( 0.0, 0.0, -climbRate );
            mr.computeForceAndMoment( vel_bas,
                                      mc::Vector3(),
                                      mc::Vector3(),
                                      mc::Vector3(),
                                      vel_bas,
                                      mc::Vector3(),
                                      mc::Vector3( 0.0, 0.0, gravAcc ),
                                      airDensity );

            // momentum theory
            // reference values calculated with momentum theory
            double vc_vi0 = climbRate / mr.getVel_i0();

            double mu_c =  climbRate / rotor_omegaR;
            double mu_d = -climbRate / rotor_omegaR;

            // induced velocity
            double lambda_i = 0.0;
            if ( vc_vi0 >= -1.0 )
            {
                // momentum theory climb
                lambda_i = -mu_c / 2.0 + sqrt( pow( mu_c / 2.0, 2.0) + pow( mr.getLambda_i0(), 2.0 ) );
            }
            else if ( vc_vi0 < -2.0 )
            {
                // momentum theory descend
                lambda_i = mu_d / 2.0 - sqrt( pow( mu_d / 2.0, 2.0) - pow( mr.getLambda_i0(), 2.0 ) );
            }
            else
            {
                // Johnson: Helicopter Theory, p.106
                double mu_c_norm = mu_c / mr.getLambda_i0();
                lambda_i = mu_c_norm * mr.getLambda_i0() * ( 0.373*mu_c_norm*mu_c_norm - 1.991 );
            }
            double vel_i = lambda_i * rotor_omegaR;

            // thrust
            double thrust = std::numeric_limits<double>::quiet_NaN();

            if ( vc_vi0 > -1.0 )
            {
                thrust = 2.0 * airDensity * rotor_area * ( climbRate + vel_i ) * vel_i;
            }
            else if ( vc_vi0 < -2.0 )
            {
                thrust = 2.0 * airDensity * rotor_area * ( fabs( climbRate ) - vel_i ) * vel_i;
            }

            file << climbRate;
            file << "\t";
            file << mr.getVel_i0();
            file << "\t";
            file << mr.getVel_i();
            file << "\t";
            file << mr.getLambda_i();
            file << "\t";
            file << ( climbRate / mr.getVel_i0() );
            file << "\t";
            file << ( mr.getVel_i() / mr.getVel_i0() );
            file << "\t";
            file << mr.getThrust();
            file << "\t";
            file << mr.getTorque();

            file << "\t";
            file << vel_i;
            file << "\t";
            file << lambda_i;
            file << "\t";
            file << ( vel_i / mr.getVel_i0() );
            file << "\t";
            file << thrust;
            file << std::endl;

            // climb rate increment
            climbRate += climbRate_step;
        }
        while ( climbRate <= climbRate_max );

        file.close();
    }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, SaveResultsToFile2)
{
    std::ofstream file;
    file.open( "../main_rotor2.csv", std::ios_base::out );

    if ( file.is_open() )
    {
        file << R"##("vc")##";
        file << "\t";
        file << R"##("vi0")##";
        file << "\t";
        file << R"##("vi")##";
        file << "\t";
        file << R"##("lambda_i")##";
        file << "\t";
        file << R"##("vc_vi0")##";
        file << "\t";
        file << R"##("vi_vi0")##";
        file << "\t";
        file << R"##("thrust")##";
        file << "\t";
        file << R"##("torque")##";

        file << "\t";
        file << R"##("mt_vi")##";
        file << "\t";
        file << R"##("mt_lambda_i")##";
        file << "\t";
        file << R"##("mt_vi_vi0")##";
        file << "\t";
        file << R"##("mt_thrust")##";
        file << std::endl;

        TestMainRotor::MainRotorAdapter mr;
        mr.setData( getMainRotorData() );

        double collective = mc::Units::deg2rad( 6.0 );
        mr.update( rotor_omega, 0.0, collective, 0.0, 0.0 );

        const double climbRate_min  = -30.0;
        const double climbRate_max  =  10.0;
        const double climbRate_step =   0.1;

        double climbRate = climbRate_min;

        do
        {
            double mu_c =  climbRate / rotor_omegaR;
            double mu_d = -climbRate / rotor_omegaR;

            // main rotor
            double r2 = mr.getData().r * mr.getData().r;
            double r4 = r2 * r2;
            double ib = mr.getData().blade_mass * r2 / 3.0;
            double gamma = airDensity * mr.getData().a * mr.getData().c * r4 / ib;
            mr.updateFlappingAnglesThrustCoefsAndVelocity( 0.0, 0.0, mu_d,
                                                           0.0, 0.0, gravAcc,
                                                           gamma );

            double vi0 = mr.getLambda_i0() * rotor_omegaR;

            // momentum theory
            // reference values calculated with momentum theory
            double vc_vi0 = climbRate / mr.getVel_i0();

            // induced velocity
            double lambda_i = 0.0;
            if ( vc_vi0 >= -1.0 )
            {
                // momentum theory climb
                lambda_i = -mu_c / 2.0 + sqrt( pow( mu_c / 2.0, 2.0) + pow( mr.getLambda_i0(), 2.0 ) );
            }
            else if ( vc_vi0 < -2.0 )
            {
                // momentum theory descend
                lambda_i = mu_d / 2.0 - sqrt( pow( mu_d / 2.0, 2.0) - pow( mr.getLambda_i0(), 2.0 ) );
            }
            else
            {
                // Johnson: Helicopter Theory, p.106
                double mu_c_norm = mu_c / mr.getLambda_i0();
                lambda_i = mu_c_norm * mr.getLambda_i0() * ( 0.373*mu_c_norm*mu_c_norm - 1.991 );
            }
            double vel_i = lambda_i * rotor_omegaR;

            // thrust
            double thrust = std::numeric_limits<double>::quiet_NaN();

            if ( vc_vi0 > -1.0 )
            {
                thrust = 2.0 * airDensity * rotor_area * ( climbRate + vel_i ) * vel_i;
            }
            else if ( vc_vi0 < -2.0 )
            {
                thrust = 2.0 * airDensity * rotor_area * ( fabs( climbRate ) - vel_i ) * vel_i;
            }

            file << climbRate;
            file << "\t";
            file << vi0;
            file << "\t";
            file << mr.getVel_i();
            file << "\t";
            file << mr.getLambda_i();
            file << "\t";
            file << ( climbRate / vi0 );
            file << "\t";
            file << ( mr.getLambda_i() / mr.getLambda_i0() );
            file << "\t";
            file << mr.getThrust();
            file << "\t";
            file << mr.getTorque();

            file << "\t";
            file << vel_i;
            file << "\t";
            file << lambda_i;
            file << "\t";
            file << ( vel_i / vi0 );
            file << "\t";
            file << thrust;
            file << std::endl;

            // climb rate increment
            climbRate += climbRate_step;
        }
        while ( climbRate <= climbRate_max );

        file.close();
    }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, CanConstruct)
{
    mc::MainRotor *rotor = nullptr;
    EXPECT_NO_THROW( rotor = new mc::MainRotor() );
    delete rotor;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, CanDestruct)
{
    mc::MainRotor *rotor = new mc::MainRotor();
    EXPECT_NO_THROW( delete rotor );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, CanInstantiate)
{
    mc::MainRotor rotor;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, CanSimulateComparedToMomentumTheory)
{
    mc::MainRotor mr;
    mr.setData( getMainRotorData() );

    double collective = mc::Units::deg2rad( 6.0 );
    mr.update( rotor_omega, 0.0, collective, 0.0, 0.0 );

    const double climbRate_min  = -30.0;
    const double climbRate_max  =  10.0;
    const double climbRate_step =   0.1;

    double climbRate = climbRate_min;

    do
    {
        mc::Vector3 vel_bas( 0.0, 0.0, -climbRate );

        mr.computeForceAndMoment( vel_bas,
                                  mc::Vector3(),
                                  mc::Vector3(),
                                  mc::Vector3(),
                                  vel_bas,
                                  mc::Vector3(),
                                  mc::Vector3( 0.0, 0.0, gravAcc ),
                                  airDensity );


        // reference values calculated with momentum theory
        double vc_vi0 = climbRate / mr.getVel_i0();

        double mu_c =  climbRate / rotor_omegaR;
        double mu_d = -climbRate / rotor_omegaR;

        // induced velocity
        double lambda_i = 0.0;
        if ( vc_vi0 >= -1.0 )
        {
            // momentum theory climb
            lambda_i = -mu_c / 2.0 + sqrt( pow( mu_c / 2.0, 2.0) + pow( mr.getLambda_i0(), 2.0 ) );
        }
        else if ( vc_vi0 < -2.0 )
        {
            // momentum theory descend
            lambda_i = mu_d / 2.0 - sqrt( pow( mu_d / 2.0, 2.0) - pow( mr.getLambda_i0(), 2.0 ) );
        }
        else
        {
            // Johnson: Helicopter Theory, p.106
            double mu_c_norm = mu_c / mr.getLambda_i0();
            lambda_i = mu_c_norm * mr.getLambda_i0() * ( 0.373*mu_c_norm*mu_c_norm - 1.991 );
        }
        double vel_i = lambda_i * rotor_omegaR;

        // 5% tolerance
        double tol_lambda_i = 0.05 * fabs( lambda_i );
        double tol_vel_i    = 0.05 * fabs( vel_i    );

        // 5% tolerance

        EXPECT_NEAR( mr.getLambda_i() , lambda_i , tol_lambda_i );
        EXPECT_NEAR( mr.getVel_i()    , vel_i    , tol_vel_i    );

        if ( vc_vi0 > -1.0 || vc_vi0 < -2.0 )
        {
            // thrust
            double thrust = std::numeric_limits<double>::quiet_NaN();

            if ( vc_vi0 > -1.0 )
            {
                thrust = 2.0 * airDensity * rotor_area * ( climbRate + vel_i ) * vel_i;
            }
            else if ( vc_vi0 < -2.0 )
            {
                thrust = 2.0 * airDensity * rotor_area * ( fabs( climbRate ) - vel_i ) * vel_i;
            }

            // 5% tolerance
            double tol_thrust = 0.05 * fabs( thrust );

            EXPECT_NEAR( mr.getThrust(), thrust, tol_thrust );
        }

        // climb rate increment
        climbRate += climbRate_step;
    }
    while ( climbRate <= climbRate_max );
}
