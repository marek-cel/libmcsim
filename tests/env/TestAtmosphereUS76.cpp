#include <gtest/gtest.h>

#include <iostream>

#include <mcsim/env/AtmosphereUS76.h>

////////////////////////////////////////////////////////////////////////////////

class TestAtmosphereUS76 : public ::testing::Test
{
protected:

    TestAtmosphereUS76() {}
    virtual ~TestAtmosphereUS76() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAtmosphereUS76, CanAssertTrue)
{
    mc::AtmosphereUS76 atm;

    atm.update( 0.0 );

    std::cout << "t= " << atm.getTemperature() << std::endl;
    std::cout << "p= " << atm.getPressure() << std::endl;
    std::cout << "rho= " << atm.getDensity() << std::endl;
    std::cout << "a= " << atm.getSpeedOfSound() << std::endl;
    std::cout << "mu= " << atm.getDynViscosity() << std::endl;
    std::cout << "nu= " << atm.getKinViscosity() << std::endl;

    ASSERT_TRUE( true );
}
