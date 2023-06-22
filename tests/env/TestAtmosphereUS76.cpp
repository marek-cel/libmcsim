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

    atm.Update(0.0);

    std::cout << "t= " << atm.temperature() << std::endl;
    std::cout << "p= " << atm.pressure() << std::endl;
    std::cout << "rho= " << atm.density() << std::endl;
    std::cout << "a= " << atm.speed_of_sound() << std::endl;
    std::cout << "mu= " << atm.dyn_viscosity() << std::endl;
    std::cout << "nu= " << atm.kin_viscosity() << std::endl;

    ASSERT_TRUE( true );
}
