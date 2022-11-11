#include <gtest/gtest.h>

#include <mcsim/aero/AeroAngles.h>

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
    EXPECT_NEAR( beta,  M_PI_2, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( 0.0, -10.0, 0.0 ) );
    EXPECT_NEAR( beta, -M_PI_2, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( 10.0, 10.0, 0.0 ) );
    EXPECT_NEAR( beta,  M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( 10.0, -10.0, 0.0 ) );
    EXPECT_NEAR( beta, -M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( -10.0, 10.0, 0.0 ) );
    EXPECT_NEAR( beta,  M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( mc::Vector3( -10.0, -10.0, 0.0 ) );
    EXPECT_NEAR( beta, -M_PI_4, 1.0e-6 );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroAngles, CanGetSideslipAngleFromUV)
{
    double beta = 0.0;

    beta = mc::getSideslipAngle( 0.0, 0.0 );
    EXPECT_NEAR( beta, 0.0, 1.0e-6 );

    beta = mc::getSideslipAngle( 10.0, 0.0 );
    EXPECT_NEAR( beta,  M_PI_2, 1.0e-6 );

    beta = mc::getSideslipAngle( -10.0, 0.0 );
    EXPECT_NEAR( beta, -M_PI_2, 1.0e-6 );

    beta = mc::getSideslipAngle( 10.0, 10.0 );
    EXPECT_NEAR( beta,  M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( -10.0, 10.0 );
    EXPECT_NEAR( beta, -M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( 10.0, -10.0 );
    EXPECT_NEAR( beta,  M_PI_4, 1.0e-6 );

    beta = mc::getSideslipAngle( -10.0, -10.0 );
    EXPECT_NEAR( beta, -M_PI_4, 1.0e-6 );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroAngles, CanGetAero2BAS)
{
    mc::Vector3 v_bas;

    double aoa = 0.0;
    double sslip = 0.0;

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 1.0, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 0.0, 1.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = M_PI_4;
    sslip = 0.0;

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( M_SQRT2, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 0.0, M_SQRT2 );
    EXPECT_NEAR( v_bas.x(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = -M_PI_4;
    sslip = 0.0;

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( M_SQRT2, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  1.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 0.0, M_SQRT2 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = 0.0;
    sslip = M_PI_4;

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( M_SQRT2, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, M_SQRT2, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 0.0, 1.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = 0.0;
    sslip = -M_PI_4;

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( M_SQRT2, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, M_SQRT2, 0.0 );
    EXPECT_NEAR( v_bas.x(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 0.0, 1.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = 0.0;
    sslip = M_PI_2;

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 1.0, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 0.0, 1.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = 0.0;
    sslip = -M_PI_2;

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 1.0, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 0.0, 1.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = M_PI_4;
    sslip = M_PI_4;

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 1.0, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -0.5      , 1.0e-6 );
    EXPECT_NEAR( v_bas.y(), -0.707107 , 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -0.5      , 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -0.5      , 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.707107 , 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -0.5      , 1.0e-6 );

    v_bas = mc::getAero2BAS( aoa, sslip ) * mc::Vector3( 0.0, 0.0, 1.0 );
    EXPECT_NEAR( v_bas.x(),  0.707107 , 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0      , 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -0.707107 , 1.0e-6 );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroAngles, CanGetStab2BAS)
{
    mc::Vector3 v_bas;

    double aoa = 0.0;

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( 1.0, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( 0.0, 0.0, 1.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = M_PI_4;

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( M_SQRT2, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( 0.0, 0.0, M_SQRT2 );
    EXPECT_NEAR( v_bas.x(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );

    aoa = -M_PI_4;

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( M_SQRT2, 0.0, 0.0 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  1.0, 1.0e-6 );

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( 0.0, 1.0, 0.0 );
    EXPECT_NEAR( v_bas.x(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(),  0.0, 1.0e-6 );

    v_bas = mc::getStab2BAS( aoa ) * mc::Vector3( 0.0, 0.0, M_SQRT2 );
    EXPECT_NEAR( v_bas.x(), -1.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.y(),  0.0, 1.0e-6 );
    EXPECT_NEAR( v_bas.z(), -1.0, 1.0e-6 );
}
