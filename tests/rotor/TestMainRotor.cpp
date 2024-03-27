#include <gtest/gtest.h>

#include <fstream>

#include <mcutils/misc/Units.h>
#include <mcsim/rotor/MainRotor.h>

#include <rotor/MomentumTheory.h>

////////////////////////////////////////////////////////////////////////////////
// TESTS UTILITIES
////////////////////////////////////////////////////////////////////////////////

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
        _data.ccw = true;

        _data.nb = 4;

        _data.blade_mass = 116.5;

        _data.r = ROTOR_RADIUS;
        _data.c = 0.53;
        _data.e = 0.38;

        _data.a = 5.73;
        _data.b = 0.97;

        _data.delta_0 = 0.0;
        _data.delta_2 = 0.0;

        _data.beta_max = mc::Units::deg2rad(20.0);

        _data.ct_max = DBL_MAX;
        _data.ch_max = DBL_MAX;
        _data.cq_max = DBL_MAX;

        _data.thrust_factor = 1.0;
        _data.hforce_factor = 1.0;
        _data.torque_factor = 1.0;

        _data.vrs_thrust_factor = 1.0;
        _data.vrs_torque_factor = 1.0;

        UpdateDataDerivedVariables();
    }

    const Data* GetData() const override
    {
        return &_data;
    }

    double GetInGroundEffectThrustCoef(double, double, double) override
    {
        return 0.0;
    }

    double GetVortexRingInfluenceCoef(double, double) override
    {
        return 0.0;
    }

private:

    Data _data;
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

class ResultsWriter
{
public:
    ResultsWriter(const char* file_name)
    {
        file.open(file_name, std::ios_base::out);
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
            file << R"##("mt_lambda_i")##";
            file << "\t";
            file << R"##("mt_vi_vi0")##";
            file << "\t";
            file << R"##("mt_thrust")##";

            file << std::endl;
        }
    }

    ~ResultsWriter()
    {
        if ( file.is_open() )
        {
            file.close();
        }
    }

    void WriteResults(double vc, double vi0,
                      const MainRotorAdapter* mr,
                      const MomentumTheoryAdapter* mt)
    {
        if ( file.is_open() )
        {
            file << vc;
            file << "\t";
            file << vi0;

            file << "\t";
            file << mr->vel_i();
            file << "\t";
            file << mr->lambda_i();
            file << "\t";
            file << (vc / vi0);
            file << "\t";
            file << (mr->lambda_i() / mr->lambda_i0());
            file << "\t";
            file << mr->thrust();
            file << "\t";
            file << mr->torque();

            file << "\t";
            file << mt->lambda_i();
            file << "\t";
            file << mt->vi_vi0();
            file << "\t";
            file << mt->thrust();

            file << std::endl;
        }
    }

private:
    std::ofstream file;
};

////////////////////////////////////////////////////////////////////////////////
// TESTS START HERE
////////////////////////////////////////////////////////////////////////////////

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

    //ResultsWriter rw("../main_rotor1.csv");
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

        //rw.WriteResults(climb_rate, mr.vel_i0(), &mr, &mt);

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

    EXPECT_DOUBLE_EQ(mr.GetData()->r_hub_bas.x(), 0.0);
    EXPECT_DOUBLE_EQ(mr.GetData()->r_hub_bas.y(), 0.0);
    EXPECT_DOUBLE_EQ(mr.GetData()->r_hub_bas.z(), 0.0);

    EXPECT_DOUBLE_EQ(mr.GetData()->a_hub_bas.phi(), 0.0);
    EXPECT_DOUBLE_EQ(mr.GetData()->a_hub_bas.tht(), 0.0);
    EXPECT_DOUBLE_EQ(mr.GetData()->a_hub_bas.psi(), 0.0);

    EXPECT_TRUE(mr.GetData()->ccw);

    EXPECT_EQ(mr.GetData()->nb, 4);

    EXPECT_DOUBLE_EQ(mr.GetData()->blade_mass, 116.5);

    EXPECT_DOUBLE_EQ(mr.GetData()->r, ROTOR_RADIUS);
    EXPECT_DOUBLE_EQ(mr.GetData()->c, 0.53);
    EXPECT_DOUBLE_EQ(mr.GetData()->e, 0.38);

    EXPECT_DOUBLE_EQ(mr.GetData()->a, 5.73);
    EXPECT_DOUBLE_EQ(mr.GetData()->b, 0.97);

    EXPECT_DOUBLE_EQ(mr.GetData()->delta_0, 0.0);
    EXPECT_DOUBLE_EQ(mr.GetData()->delta_2, 0.0);

    EXPECT_DOUBLE_EQ(mr.GetData()->beta_max, mc::Units::deg2rad(20.0));

    EXPECT_DOUBLE_EQ(mr.GetData()->ct_max, DBL_MAX);
    EXPECT_DOUBLE_EQ(mr.GetData()->ch_max, DBL_MAX);
    EXPECT_DOUBLE_EQ(mr.GetData()->cq_max, DBL_MAX);

    EXPECT_DOUBLE_EQ(mr.GetData()->thrust_factor, 1.0);
    EXPECT_DOUBLE_EQ(mr.GetData()->hforce_factor, 1.0);
    EXPECT_DOUBLE_EQ(mr.GetData()->torque_factor, 1.0);

    EXPECT_DOUBLE_EQ(mr.GetData()->vrs_thrust_factor, 1.0);
    EXPECT_DOUBLE_EQ(mr.GetData()->vrs_torque_factor, 1.0);
}
