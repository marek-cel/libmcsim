#include <gtest/gtest.h>

#include <mcutils/misc/Units.h>
#include <mcsim/rotor/TailRotor.h>

#include <rotor/MomentumTheory.h>

#define AIR_DENSITY 1.225
#define GRAV_ACC 9.80665
#define ALT_OUTSIDE_IGE 1000.0

#define ROTOR_RADIUS 1.675
#define ROTOR_OMEGA (2 * M_PI * 1190.0 / 60.0)

class TailRotorAdapter : public mc::TailRotor
{
public:

    void InitData()
    {
        data_.nb = 4;

        data_.blade_mass = 116.5;

        data_.r = ROTOR_RADIUS;
        data_.c = 0.25;

        data_.a = 5.73;
        data_.b = 0.92;

        data_.delta_0 = 0.0;
        data_.delta_2 = 0.0;

        data_.ct_max = DBL_MAX;
        data_.cq_max = DBL_MAX;

        data_.thrust_factor = 1.0;
        data_.torque_factor = 1.0;

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

class TestTailRotor : public ::testing::Test
{
protected:

    TestTailRotor() {}
    virtual ~TestTailRotor() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestTailRotor, CanConstruct)
{
    TailRotorAdapter* rotor = nullptr;
    EXPECT_NO_THROW(rotor = new TailRotorAdapter());
    delete rotor;
}

TEST_F(TestTailRotor, CanDestruct)
{
    TailRotorAdapter* rotor = new TailRotorAdapter();
    EXPECT_NO_THROW(delete rotor);
}

TEST_F(TestTailRotor, CanInstantiate)
{
    TailRotorAdapter rotor;
}

TEST_F(TestTailRotor, CanSimulateComparedToMomentumTheory)
{
    MomentumTheoryAdapter mt;
    mt.InitData();

    TailRotorAdapter tr;
    tr.InitData();

    double collective = mc::Units::deg2rad(6.0);
    tr.Update(ROTOR_OMEGA, collective);

    const double climb_rate_min  = -30.0;
    const double climb_rate_max  =  10.0;
    const double climb_rate_step =   0.1;
    double climb_rate = climb_rate_min;

    do
    {
        mc::Vector3 vel_bas(0.0, 0.0, -climb_rate);

        tr.UpdateForceAndMoment(vel_bas,
                                mc::Vector3(),
                                AIR_DENSITY);

        mt.Update(climb_rate, tr.vel_i0(), AIR_DENSITY);

        // 5% tolerance
        double tol_lambda_i = std::max(0.05 * fabs(mt.lambda_i()), 1.0e-9);
        EXPECT_NEAR(tr.lambda_i(), mt.lambda_i(), tol_lambda_i);

        if ( mc::IsValid(mt.thrust()) )
        {
            // 5% tolerance
            double tol_thrust = 0.05 * fabs(mt.thrust());
            EXPECT_NEAR(tr.thrust(), mt.thrust(), tol_thrust);
        }

        // climb rate increment
        climb_rate += climb_rate_step;
    }
    while ( climb_rate <= climb_rate_max );
}