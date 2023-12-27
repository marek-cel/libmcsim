#include <gtest/gtest.h>

#include <cmath>

#include <mcsim/rotor/RotorUtils.h>

#include <CsvFileReader.h>

#include <iostream>

////////////////////////////////////////////////////////////////////////////////

class TestRotorUtils : public ::testing::Test
{
protected:

    TestRotorUtils() {}
    virtual ~TestRotorUtils() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestRotorUtils, CanGetVortexRingK0)
{
    std::vector<double> vz_norm;
    std::vector<double> k0_ref;

    // expected values from
    // Toropov, Stepanov: Modeling of Helicopter Flight Imitation in the Vortex Ring State
    ReadCsvDataFromFile("../tests/rotor/data/k0.csv", &vz_norm, &k0_ref);

    for ( unsigned int i = 0; i < vz_norm.size(); i++ )
    {
        double k0 = mc::GetVortexRingK0(-vz_norm.at(i));
        double tol = std::max(0.05 * k0_ref.at(i), 0.05);
        EXPECT_NEAR(k0, k0_ref.at(i), tol);
    }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestRotorUtils, CanGetVortexRingKv)
{
    std::vector<double> vx_norm;
    std::vector<double> kv_ref;

    // expected values from
    // Toropov, Stepanov: Modeling of Helicopter Flight Imitation in the Vortex Ring State
    ReadCsvDataFromFile("../tests/rotor/data/kv.csv", &vx_norm, &kv_ref);

    for ( unsigned int i = 0; i < vx_norm.size(); i++ )
    {
        double kv = mc::GetVortexRingKv(vx_norm.at(i));
        double tol = std::max(0.1 * kv_ref.at(i), 0.1);
        EXPECT_NEAR(kv, kv_ref.at(i), tol);
    }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestRotorUtils, CanGetInGroundEffectThrustCoef)
{
    std::vector<double> v_vi;
    std::vector<double> zg_r_06;
    std::vector<double> zg_r_08;
    std::vector<double> zg_r_10;
    std::vector<double> zg_r_20;

    // expected values from
    // Padfield G.: Helicopter Flight Dynamics, 2007, p.142
    ReadCsvDataFromFile("../tests/rotor/data/ige.csv",
                        &v_vi, &zg_r_06, &zg_r_08, &zg_r_10, &zg_r_20);

    for ( unsigned int i = 0; i < v_vi.size(); i++ )
    {
        double ige_06 = mc::GetInGroundEffectThrustCoef(0.6, v_vi.at(i), 1.0, 1.0);
        double ige_08 = mc::GetInGroundEffectThrustCoef(0.8, v_vi.at(i), 1.0, 1.0);
        double ige_10 = mc::GetInGroundEffectThrustCoef(1.0, v_vi.at(i), 1.0, 1.0);
        double ige_20 = mc::GetInGroundEffectThrustCoef(2.0, v_vi.at(i), 1.0, 1.0);

        double tol_06 = std::max(0.1 * ige_06, 0.1);
        double tol_08 = std::max(0.1 * ige_08, 0.1);
        double tol_10 = std::max(0.1 * ige_10, 0.1);
        double tol_20 = std::max(0.1 * ige_20, 0.1);

        EXPECT_NEAR(ige_06, zg_r_06.at(i), tol_06);
        EXPECT_NEAR(ige_08, zg_r_08.at(i), tol_08);
        EXPECT_NEAR(ige_10, zg_r_10.at(i), tol_10);
        EXPECT_NEAR(ige_20, zg_r_20.at(i), tol_20);
    }
}
