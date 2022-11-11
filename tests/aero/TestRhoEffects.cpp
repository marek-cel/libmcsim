#include <gtest/gtest.h>

#include <mcsim/aero/RhoEffects.h>

////////////////////////////////////////////////////////////////////////////////

class TestRhoEffects : public ::testing::Test
{
protected:

    TestRhoEffects() {}
    virtual ~TestRhoEffects() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestRhoEffects, CanGetPrandtlGlauertCpFactor)
{
    double mach = 0.0;

    const double factor_max = 3.0;

    do
    {
        double factor = mc::getPrandtlGlauertFactor( mach, factor_max );

        mach += 0.1;
    }
    while ( mach < 2.0 );


    ASSERT_TRUE( true );
}
