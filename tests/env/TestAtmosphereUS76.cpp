#include <gtest/gtest.h>

#include <iostream>

#include <mcsim/env/AtmosphereUS76.h>
#include <mcutils/misc/Check.h>

#include <CsvFileReader.h>

class TestAtmosphereUS76 : public ::testing::Test
{
protected:

    TestAtmosphereUS76() {}
    virtual ~TestAtmosphereUS76() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestAtmosphereUS76, CanInstantiate)
{
    mc::AtmosphereUS76 atm;
}

TEST_F(TestAtmosphereUS76, CanGetDensityAltitude)
{
    mc::AtmosphereUS76 atm;

    EXPECT_NEAR(     0.0, atm.GetDensityAltitude( 101325.0, 288.15,     0.0 ), 1.0 );
    EXPECT_NEAR( 11000.0, atm.GetDensityAltitude(  22632.1, 216.65, 11000.0 ), 1.0 );
}

TEST_F(TestAtmosphereUS76, CanCalculate)
{
    std::vector<double> h_ref;
    std::vector<double> t_ref;
    std::vector<double> p_ref;
    std::vector<double> rho_ref;
    std::vector<double> cs_ref;
    std::vector<double> mu_ref;

    // expected values from https://www.digitaldutch.com/atmoscalc/table.htm
    ReadCsvDataFromFile("../tests/env/data/test_env_ussa1976.csv",
                        &h_ref, &t_ref, &p_ref, &rho_ref, &cs_ref, &mu_ref);

    EXPECT_GT(h_ref   .size(), 0) << "No reference data.";
    EXPECT_GT(t_ref   .size(), 0) << "No reference data.";
    EXPECT_GT(p_ref   .size(), 0) << "No reference data.";
    EXPECT_GT(rho_ref .size(), 0) << "No reference data.";
    EXPECT_GT(cs_ref  .size(), 0) << "No reference data.";
    EXPECT_GT(mu_ref  .size(), 0) << "No reference data.";

    EXPECT_EQ(h_ref.size(), t_ref   .size()) << "Reference data corrupted.";
    EXPECT_EQ(h_ref.size(), p_ref   .size()) << "Reference data corrupted.";
    EXPECT_EQ(h_ref.size(), rho_ref .size()) << "Reference data corrupted.";
    EXPECT_EQ(h_ref.size(), cs_ref  .size()) << "Reference data corrupted.";
    EXPECT_EQ(h_ref.size(), mu_ref  .size()) << "Reference data corrupted.";

    mc::AtmosphereUS76 atm;

    for ( unsigned int i = 0; i < h_ref.size(); i++ )
    {
        double h = h_ref.at(i);
        
        atm.Update(h);

        double t   = atm.temperature();
        double p   = atm.pressure();
        double rho = atm.density();
        double cs  = atm.speed_of_sound();
        double mu  = atm.dyn_viscosity();

        double tol_t   = 0.01 * t_ref   .at(i);
        double tol_p   = 0.01 * p_ref   .at(i);
        double tol_rho = 0.01 * rho_ref .at(i);
        double tol_cs  = 0.01 * cs_ref  .at(i);
        double tol_mu  = 0.1  * mu_ref  .at(i);

        EXPECT_NEAR( t   , t_ref   .at(i), tol_t   ) << "Mismatch at altitude= " << h;
        EXPECT_NEAR( p   , p_ref   .at(i), tol_p   ) << "Mismatch at altitude= " << h;
        EXPECT_NEAR( rho , rho_ref .at(i), tol_rho ) << "Mismatch at altitude= " << h;
        EXPECT_NEAR( cs  , cs_ref  .at(i), tol_cs  ) << "Mismatch at altitude= " << h;
        EXPECT_NEAR( mu  , mu_ref  .at(i), tol_mu  ) << "Mismatch at altitude= " << h;
    }
}

TEST_F(TestAtmosphereUS76, CanCalculateOutOfRange)
{
    mc::AtmosphereUS76 atm;
    atm.Update(1.0e7);

    double t   = atm.temperature();
    double p   = atm.pressure();
    double rho = atm.density();
    double cs  = atm.speed_of_sound();
    double mu  = atm.dyn_viscosity();

    EXPECT_TRUE(mc::IsValid(t));
    EXPECT_TRUE(mc::IsValid(p));
    EXPECT_TRUE(mc::IsValid(rho));
    EXPECT_TRUE(mc::IsValid(cs));
    EXPECT_TRUE(mc::IsValid(mu));
}

TEST_F(TestAtmosphereUS76, CanSetSeaLevelPressure)
{
    mc::AtmosphereUS76 atm;

    EXPECT_NO_THROW(atm.set_sl_pressure(100000.0));
    EXPECT_DOUBLE_EQ(atm.sl_pressure(), 100000.0);
}

TEST_F(TestAtmosphereUS76, CanSetSeaLevelPressureOutOfRange)
{
    mc::AtmosphereUS76 atm;

    EXPECT_NO_THROW(atm.set_sl_pressure(1000.0));
    EXPECT_DOUBLE_EQ(atm.sl_pressure(), mc::AtmosphereUS76::kStdSlPress);
}

TEST_F(TestAtmosphereUS76, CanSetSeaLevelTemperature)
{
    mc::AtmosphereUS76 atm;

    EXPECT_NO_THROW(atm.set_sl_temperature(300.0));
    EXPECT_DOUBLE_EQ(atm.sl_temperature(), 300.0);
}

TEST_F(TestAtmosphereUS76, CanSetSeaLevelTemperatureOutOfRange)
{
    mc::AtmosphereUS76 atm;

    EXPECT_NO_THROW(atm.set_sl_temperature(10.0));
    EXPECT_DOUBLE_EQ(atm.sl_temperature(), mc::AtmosphereUS76::kStdSlTemp);
}
