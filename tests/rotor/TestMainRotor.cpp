#include <gtest/gtest.h>

#include <fstream>

#include <mcutils/misc/Units.h>
#include <mcsim/rotor/MainRotor.h>

#include <rotor/MomentumTheory.h>

#define AIR_DENSITY 1.225
#define GRAV_ACC 9.80665
#define ALT_OUTSIDE_IGE 1000.0

#define ROTOR_RADIUS 8.18
#define ROTOR_OMEGA (2 * M_PI *  258.0 / 60.0)

class MainRotorAdapter : public mc::MainRotor
{
public:

    void InitData()
    {
        data_.ccw = true;

        data_.nb = 4;

        data_.blade_mass = 116.5;

        data_.r = ROTOR_RADIUS;
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

private:

    Data data_;
};

class MomentumTheoryAdapter : public MomentumTheory
{
public:

    void InitData()
    {
        set_radius(ROTOR_RADIUS);
        set_omega(ROTOR_OMEGA);
    }
};

class TestMainRotor : public ::testing::Test
{
protected:

    TestMainRotor() {}
    virtual ~TestMainRotor() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestMainRotor, CanInstantiate)
{
    MainRotorAdapter mr;
}

TEST_F(TestMainRotor, CanSimulateComparedToMomentumTheory)
{
    MomentumTheoryAdapter mt;
    mt.InitData();

    MainRotorAdapter mr;
    mr.InitData();

    double collective = mc::Units::deg2rad(6.0);
    mr.Update(ROTOR_OMEGA, collective, 0.0, 0.0);

    const double climb_rate_min  = -30.0;
    const double climb_rate_max  =  10.0;
    const double climb_rate_step =   0.1;
    double climb_rate = climb_rate_min;

    do
    {
        mc::Vector3 vel_bas(0.0, 0.0, -climb_rate);

        mr.UpdateForceAndMoment(vel_bas,
                                mc::Vector3(),
                                mc::Vector3(),
                                mc::Vector3(),
                                vel_bas,
                                mc::Vector3(),
                                mc::Vector3(0.0, 0.0, GRAV_ACC),
                                AIR_DENSITY,
                                ALT_OUTSIDE_IGE);

        mt.Update(climb_rate, mr.vel_i0(), AIR_DENSITY);

        // 5% tolerance
        double tol_lambda_i = std::max(0.05 * fabs(mt.lambda_i()), 1.0e-9);
        EXPECT_NEAR(mr.lambda_i(), mt.lambda_i(), tol_lambda_i);

        if ( mc::IsValid(mt.thrust()) )
        {
            // 5% tolerance
            double tol_thrust = 0.05 * fabs(mt.thrust());
            EXPECT_NEAR(mr.thrust(), mt.thrust(), tol_thrust);
        }

        // climb rate increment
        climb_rate += climb_rate_step;
    }
    while ( climb_rate <= climb_rate_max );
}

TEST_F(TestMainRotor, CanGetData)
{
    MainRotorAdapter mr;
    mr.InitData();

    EXPECT_DOUBLE_EQ(mr.data().r_hub_bas.x(), 0.0);
    EXPECT_DOUBLE_EQ(mr.data().r_hub_bas.y(), 0.0);
    EXPECT_DOUBLE_EQ(mr.data().r_hub_bas.z(), 0.0);

    EXPECT_DOUBLE_EQ(mr.data().a_hub_bas.phi(), 0.0);
    EXPECT_DOUBLE_EQ(mr.data().a_hub_bas.tht(), 0.0);
    EXPECT_DOUBLE_EQ(mr.data().a_hub_bas.psi(), 0.0);

    EXPECT_TRUE(mr.data().ccw);

    EXPECT_EQ(mr.data().nb, 4);

    EXPECT_DOUBLE_EQ(mr.data().blade_mass, 116.5);

    EXPECT_DOUBLE_EQ(mr.data().r, ROTOR_RADIUS);
    EXPECT_DOUBLE_EQ(mr.data().c, 0.53);
    EXPECT_DOUBLE_EQ(mr.data().e, 0.38);

    EXPECT_DOUBLE_EQ(mr.data().a, 5.73);
    EXPECT_DOUBLE_EQ(mr.data().b, 0.97);

    EXPECT_DOUBLE_EQ(mr.data().delta_0, 0.0);
    EXPECT_DOUBLE_EQ(mr.data().delta_2, 0.0);

    EXPECT_DOUBLE_EQ(mr.data().beta_max, mc::Units::deg2rad(20.0));

    EXPECT_DOUBLE_EQ(mr.data().ct_max, DBL_MAX);
    EXPECT_DOUBLE_EQ(mr.data().ch_max, DBL_MAX);
    EXPECT_DOUBLE_EQ(mr.data().cq_max, DBL_MAX);

    EXPECT_DOUBLE_EQ(mr.data().thrust_factor, 1.0);
    EXPECT_DOUBLE_EQ(mr.data().hforce_factor, 1.0);
    EXPECT_DOUBLE_EQ(mr.data().torque_factor, 1.0);

    EXPECT_DOUBLE_EQ(mr.data().vrs_thrust_factor, 1.0);
    EXPECT_DOUBLE_EQ(mr.data().vrs_torque_factor, 1.0);
}
