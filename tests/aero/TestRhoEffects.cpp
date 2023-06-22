#include <gtest/gtest.h>

#include <cmath>

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
        double factor = mc::GetPrandtlGlauertFactor(mach, factor_max);

        // https://en.wikipedia.org/wiki/Prandtl%E2%80%93Glauert_singularity
        // cp = cp0 / sqrt( | 1 - M^2 | )
        double factor_expected = 1.0 / sqrt( fabs(1.0 - pow(mach, 2.0)) );
        factor_expected = std::min(factor_expected, factor_max);

        EXPECT_NEAR( factor, factor_expected, 1.0e-3 );


        mach += 0.1;
    }
    while ( mach < 2.0 );
}
