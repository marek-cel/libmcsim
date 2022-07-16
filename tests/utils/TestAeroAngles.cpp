#include <gtest/gtest.h>

#include <mcsim/utils/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

class TestAeroAngles : public ::testing::Test
{
protected:
    TestAeroAngles() {}
    virtual ~TestAeroAngles() {}
    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroAngles, CanGetAngleOfAttackFromVector)
{
    double aoa = 0.0;

    aoa = mc::getAngleOfAttack( mc::Vector3( 0.0, 0.0, 0.0 ) );
    EXPECT_NEAR( aoa, 0.0, 1.0e-6 );

    aoa = mc::getAngleOfAttack( mc::Vector3( 10.0, 0.0, 0.0 ) );
    EXPECT_NEAR( aoa, 0.0, 1.0e-6 );

    aoa = mc::getAngleOfAttack( mc::Vector3( -10.0, 0.0, 0.0 ) );
    EXPECT_NEAR( aoa, M_PI, 1.0e-6 );

    aoa = mc::getAngleOfAttack( mc::Vector3( 0.0, 0.0, 10.0 ) );
    EXPECT_NEAR( aoa, M_PI_2, 1.0e-6 );

    aoa = mc::getAngleOfAttack( mc::Vector3( 0.0, 0.0, -10.0 ) );
    EXPECT_NEAR( aoa, -M_PI_2, 1.0e-6 );

    aoa = mc::getAngleOfAttack( mc::Vector3( 10.0, 0.0, 10.0 ) );
    EXPECT_NEAR( aoa, M_PI_4, 1.0e-6 );

    aoa = mc::getAngleOfAttack( mc::Vector3( 10.0, 0.0, -10.0 ) );
    EXPECT_NEAR( aoa, -M_PI_4, 1.0e-6 );

    aoa = mc::getAngleOfAttack( mc::Vector3( -10.0, 0.0, 10.0 ) );
    EXPECT_NEAR( aoa, M_PI_2 + M_PI_4, 1.0e-6 );

    aoa = mc::getAngleOfAttack( mc::Vector3( -10.0, 0.0, -10.0 ) );
    EXPECT_NEAR( aoa, -M_PI_2 - M_PI_4, 1.0e-6 );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroAngles, CanGetAngleOfAttackFromUW)
{
    double aoa = 0.0;

    aoa = mc::getAngleOfAttack( 0.0, 0.0 );
    EXPECT_NEAR( aoa, 0.0, 1.0e-6 );

    aoa = mc::getAngleOfAttack( 10.0, 0.0 );
    EXPECT_NEAR( aoa, 0.0, 1.0e-6 );

    aoa = mc::getAngleOfAttack( -10.0, 0.0 );
    EXPECT_NEAR( aoa, M_PI, 1.0e-6 );

    aoa = mc::getAngleOfAttack( 0.0, 10.0 );
    EXPECT_NEAR( aoa, M_PI_2, 1.0e-6 );

    aoa = mc::getAngleOfAttack( 0.0, -10.0 );
    EXPECT_NEAR( aoa, -M_PI_2, 1.0e-6 );

    aoa = mc::getAngleOfAttack( 10.0, 10.0 );
    EXPECT_NEAR( aoa, M_PI_4, 1.0e-6 );

    aoa = mc::getAngleOfAttack( 10.0, -10.0 );
    EXPECT_NEAR( aoa, -M_PI_4, 1.0e-6 );

    aoa = mc::getAngleOfAttack( -10.0, 10.0 );
    EXPECT_NEAR( aoa, M_PI_2 + M_PI_4, 1.0e-6 );

    aoa = mc::getAngleOfAttack( -10.0, -10.0 );
    EXPECT_NEAR( aoa, -M_PI_2 - M_PI_4, 1.0e-6 );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroAngles, CanGetSideslipAngleFromVector)
{
    double beta = 0.0;

    beta = mc::getSideslipAngle( mc::Vector3( 0.0, 0.0, 0.0 ) );
    EXPECT_NEAR( beta, 0.0, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( 0.0, 10.0, 0.0 ) );
    EXPECT_NEAR( beta, M_PI_2, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( 0.0, -10.0, 0.0 ) );
    EXPECT_NEAR( beta, -M_PI_2, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( 10.0, 10.0, 0.0 ) );
    EXPECT_NEAR( beta, M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( 10.0, -10.0, 0.0 ) );
    EXPECT_NEAR( beta, -M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( -10.0, 10.0, 0.0 ) );
    EXPECT_NEAR( beta, M_PI_2 + M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( -10.0, -10.0, 0.0 ) );
    EXPECT_NEAR( beta, -M_PI_2 - M_PI_4, 1.0e-6 );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroAngles, CanGetSideslipAngleFromUV)
{
    double beta = 0.0;

    beta = mc::getSideslipAngle( 0.0, 0.0 );
    EXPECT_NEAR( beta, 0.0, 1.0e-6 );

    beta = mc::getSideslipAngle( 0.0, 10.0 );
    EXPECT_NEAR( beta, M_PI_2, 1.0e-6 );

    beta = mc::getSideslipAngle( 0.0, -10.0 );
    EXPECT_NEAR( beta, -M_PI_2, 1.0e-6 );

    beta = mc::getSideslipAngle( 10.0, 10.0 );
    EXPECT_NEAR( beta, M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( 10.0, -10.0 );
    EXPECT_NEAR( beta, -M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( -10.0, 10.0 );
    EXPECT_NEAR( beta, M_PI_2 + M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( -10.0, -10.0 );
    EXPECT_NEAR( beta, -M_PI_2 - M_PI_4, 1.0e-6 );
}
