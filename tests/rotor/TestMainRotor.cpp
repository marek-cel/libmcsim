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

        void InitData()
        {
            data_.inclination = 0.0;

            data_.ccw = true;

            data_.nb = 4;

            data_.blade_mass = 116.5;

            data_.r = TestMainRotor::rotor_radius;
            data_.c = 0.53;
            data_.e = 0.38;

            data_.a = 5.73;
            data_.b = 0.97;

            data_.delta_0 = 0.0;
            data_.delta_2 = 0.0;

            data_.beta_max = mc::Units::deg2rad(20.0);

            data_.ct_max = DBL_MAX;
            data_.ch_max = DBL_MAX;
            data_.cq_max = DBL_MAX;

            data_.thrust_factor = 1.0;
            data_.hforce_factor = 1.0;
            data_.torque_factor = 1.0;

            data_.vrs_thrust_factor = 1.0;
            data_.vrs_torque_factor = 1.0;

            UpdateDataDerivedVariables();
        }

        const Data& data() const override
        {
            return data_;
        }

        void UpdateFlappingAnglesThrustCoefsAndVelocity(double mu_x, double mu_x2, double mu_z,
                                                        double p, double q, double a_z,
                                                        double gamma)
        {
            mc::MainRotor::UpdateFlappingAnglesThrustCoefsAndVelocity(mu_x, mu_x2, mu_z,
                                                                      p, q, a_z,
                                                                      gamma);
        }

    private:

        Data data_;
    };

    static constexpr double airDensity = 1.225;
    static constexpr double gravAcc = 9.80665;

    static constexpr double rotor_radius = 8.18;
    static constexpr double rotor_omega = 2 * M_PI *  258.0 / 60.0;
    static constexpr double rotor_omegaR = rotor_omega * rotor_radius;
    static constexpr double rotor_area = M_PI * pow( rotor_radius, 2.0 );

    static constexpr double helicopter_mass   = 6000.0;
    static constexpr double helicopter_weight = helicopter_mass * gravAcc;

    TestMainRotor() {}
    virtual ~TestMainRotor() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, SaveResultsToFile1)
{
    std::ofstream file;
    file.open("../main_rotor1.csv", std::ios_base::out);

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
        mr.InitData();

        double collective = mc::Units::deg2rad(6.0);
        mr.Update(rotor_omega, 0.0, collective, 0.0, 0.0);

        const double climbRate_min  = -30.0;
        const double climbRate_max  =  10.0;
        const double climbRate_step =   0.1;

        double climbRate = climbRate_min;

        do
        {
            // main rotor
            mc::Vector3 vel_bas(0.0, 0.0, -climbRate);
            mr.ComputeForceAndMoment(vel_bas,
                                     mc::Vector3(),
                                     mc::Vector3(),
                                     mc::Vector3(),
                                     vel_bas,
                                     mc::Vector3(),
                                     mc::Vector3( 0.0, 0.0, gravAcc ),
                                     airDensity);

            // momentum theory
            // reference values calculated with momentum theory
            double vc_vi0 = climbRate / mr.vel_i0();

            double mu_c =  climbRate / rotor_omegaR;
            double mu_d = -climbRate / rotor_omegaR;

            // induced velocity
            double lambda_i = 0.0;
            if ( vc_vi0 >= -1.0 )
            {
                // momentum theory climb
                lambda_i = -mu_c / 2.0 + sqrt( pow( mu_c / 2.0, 2.0) + pow( mr.lambda_i0(), 2.0 ) );
            }
            else if ( vc_vi0 < -2.0 )
            {
                // momentum theory descend
                lambda_i = mu_d / 2.0 - sqrt( pow( mu_d / 2.0, 2.0) - pow( mr.lambda_i0(), 2.0 ) );
            }
            else
            {
                // Johnson: Helicopter Theory, p.106
                double mu_c_norm = mu_c / mr.lambda_i0();
                lambda_i = mu_c_norm * mr.lambda_i0() * ( 0.373*mu_c_norm*mu_c_norm - 1.991 );
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
            file << mr.vel_i0();
            file << "\t";
            file << mr.vel_i();
            file << "\t";
            file << mr.lambda_i();
            file << "\t";
            file << ( climbRate / mr.vel_i0() );
            file << "\t";
            file << ( mr.vel_i() / mr.vel_i0() );
            file << "\t";
            file << mr.thrust();
            file << "\t";
            file << mr.torque();

            file << "\t";
            file << vel_i;
            file << "\t";
            file << lambda_i;
            file << "\t";
            file << ( vel_i / mr.vel_i0() );
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
    file.open("../main_rotor2.csv", std::ios_base::out);

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
        mr.InitData();

        double collective = mc::Units::deg2rad(6.0);
        mr.Update(rotor_omega, 0.0, collective, 0.0, 0.0);

        const double climbRate_min  = -30.0;
        const double climbRate_max  =  10.0;
        const double climbRate_step =   0.1;

        double climbRate = climbRate_min;

        do
        {
            double mu_c =  climbRate / rotor_omegaR;
            double mu_d = -climbRate / rotor_omegaR;

            // main rotor
            double r2 = pow(mr.data().r , 2.0);
            double r4 = pow(r2, 2.0);
            double ib = mr.data().blade_mass * r2 / 3.0;
            double gamma = airDensity * mr.data().a * mr.data().c * r4 / ib;
            mr.UpdateFlappingAnglesThrustCoefsAndVelocity(0.0, 0.0, mu_d,
                                                          0.0, 0.0, gravAcc,
                                                          gamma);

            double vi0 = mr.lambda_i0() * rotor_omegaR;

            // momentum theory
            // reference values calculated with momentum theory
            double vc_vi0 = climbRate / mr.vel_i0();

            // induced velocity
            double lambda_i = 0.0;
            if ( vc_vi0 >= -1.0 )
            {
                // momentum theory climb
                lambda_i = -mu_c / 2.0 + sqrt( pow(mu_c / 2.0, 2.0) + pow(mr.lambda_i0(), 2.0) );
            }
            else if ( vc_vi0 < -2.0 )
            {
                // momentum theory descend
                lambda_i = mu_d / 2.0 - sqrt( pow(mu_d / 2.0, 2.0) - pow(mr.lambda_i0(), 2.0) );
            }
            else
            {
                // Johnson: Helicopter Theory, p.106
                double mu_c_norm = mu_c / mr.lambda_i0();
                lambda_i = mu_c_norm * mr.lambda_i0() * ( 0.373*mu_c_norm*mu_c_norm - 1.991 );
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
            file << mr.vel_i();
            file << "\t";
            file << mr.lambda_i();
            file << "\t";
            file << ( climbRate / vi0 );
            file << "\t";
            file << ( mr.lambda_i() / mr.lambda_i0() );
            file << "\t";
            file << mr.thrust();
            file << "\t";
            file << mr.torque();

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
    TestMainRotor::MainRotorAdapter *mr = nullptr;
    EXPECT_NO_THROW( mr = new TestMainRotor::MainRotorAdapter() );
    delete mr;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, CanDestruct)
{
    TestMainRotor::MainRotorAdapter *mr = new TestMainRotor::MainRotorAdapter();
    EXPECT_NO_THROW( delete mr );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, CanInstantiate)
{
    TestMainRotor::MainRotorAdapter mr;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, CanSimulateComparedToMomentumTheory)
{
    TestMainRotor::MainRotorAdapter mr;
    mr.InitData();

    double collective = mc::Units::deg2rad(6.0);
    mr.Update(rotor_omega, 0.0, collective, 0.0, 0.0);

    const double climbRate_min  = -30.0;
    const double climbRate_max  =  10.0;
    const double climbRate_step =   0.1;

    double climbRate = climbRate_min;

    do
    {
        mc::Vector3 vel_bas(0.0, 0.0, -climbRate);

        mr.ComputeForceAndMoment(vel_bas,
                                 mc::Vector3(),
                                 mc::Vector3(),
                                 mc::Vector3(),
                                 vel_bas,
                                 mc::Vector3(),
                                 mc::Vector3( 0.0, 0.0, gravAcc ),
                                 airDensity);


        // reference values calculated with momentum theory
        double vc_vi0 = climbRate / mr.vel_i0();

        double mu_c =  climbRate / rotor_omegaR;
        double mu_d = -climbRate / rotor_omegaR;

        // induced velocity
        double lambda_i = 0.0;
        if ( vc_vi0 >= -1.0 )
        {
            // momentum theory climb
            lambda_i = -mu_c / 2.0 + sqrt( pow(mu_c / 2.0, 2.0) + pow(mr.lambda_i0(), 2.0) );
        }
        else if ( vc_vi0 < -2.0 )
        {
            // momentum theory descend
            lambda_i = mu_d / 2.0 - sqrt( pow(mu_d / 2.0, 2.0) - pow(mr.lambda_i0(), 2.0) );
        }
        else
        {
            // Johnson: Helicopter Theory, p.106
            double mu_c_norm = mu_c / mr.lambda_i0();
            lambda_i = mu_c_norm * mr.lambda_i0() * ( 0.373*mu_c_norm*mu_c_norm - 1.991 );
        }
        double vel_i = lambda_i * rotor_omegaR;

        // 5% tolerance
        double tol_lambda_i = std::max( 0.05 * fabs( lambda_i ), 1.0e-9 );
        double tol_vel_i    = std::max( 0.05 * fabs( vel_i    ), 1.0e-9 );

        // 5% tolerance

        EXPECT_NEAR( mr.lambda_i() , lambda_i , tol_lambda_i );
        EXPECT_NEAR( mr.vel_i()    , vel_i    , tol_vel_i    );

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

            EXPECT_NEAR( mr.thrust(), thrust, tol_thrust );
        }

        // climb rate increment
        climbRate += climbRate_step;
    }
    while ( climbRate <= climbRate_max );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMainRotor, CanGetData)
{
    TestMainRotor::MainRotorAdapter mr;
    mr.InitData();

    EXPECT_DOUBLE_EQ( mr.data().r_hub_bas.x(), 0.0 );
    EXPECT_DOUBLE_EQ( mr.data().r_hub_bas.y(), 0.0 );
    EXPECT_DOUBLE_EQ( mr.data().r_hub_bas.z(), 0.0 );

    EXPECT_DOUBLE_EQ( mr.data().inclination, 0.0 );

    EXPECT_TRUE( mr.data().ccw );

    EXPECT_EQ( mr.data().nb, 4 );

    EXPECT_DOUBLE_EQ( mr.data().blade_mass, 116.5 );

    EXPECT_DOUBLE_EQ( mr.data().r, TestMainRotor::rotor_radius );
    EXPECT_DOUBLE_EQ( mr.data().c, 0.53 );
    EXPECT_DOUBLE_EQ( mr.data().e, 0.38 );

    EXPECT_DOUBLE_EQ( mr.data().a, 5.73 );
    EXPECT_DOUBLE_EQ( mr.data().b, 0.97 );

    EXPECT_DOUBLE_EQ( mr.data().delta_0, 0.0 );
    EXPECT_DOUBLE_EQ( mr.data().delta_2, 0.0 );

    EXPECT_DOUBLE_EQ( mr.data().beta_max, mc::Units::deg2rad(20.0) );

    EXPECT_DOUBLE_EQ( mr.data().ct_max, DBL_MAX );
    EXPECT_DOUBLE_EQ( mr.data().ch_max, DBL_MAX );
    EXPECT_DOUBLE_EQ( mr.data().cq_max, DBL_MAX );

    EXPECT_DOUBLE_EQ( mr.data().thrust_factor, 1.0 );
    EXPECT_DOUBLE_EQ( mr.data().hforce_factor, 1.0 );
    EXPECT_DOUBLE_EQ( mr.data().torque_factor, 1.0 );

    EXPECT_DOUBLE_EQ( mr.data().vrs_thrust_factor, 1.0 );
    EXPECT_DOUBLE_EQ( mr.data().vrs_torque_factor, 1.0 );
}
