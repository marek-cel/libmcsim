#include <gtest/gtest.h>

#include <mcsim/aero/MeanAeroChord.h>

////////////////////////////////////////////////////////////////////////////////

class TestMeanAeroChord : public ::testing::Test
{
protected:

    TestMeanAeroChord() {}
    virtual ~TestMeanAeroChord() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMeanAeroChord, CanGetMeanAerodynamicChordFromCrAndCt)
{
    double cr = 1.0;
    double ct = 1.0;
    double mac = mc::getMeanAerodynamicChord( cr, ct );
    EXPECT_NEAR( mac, 1.0, 1.0e-3 );

    cr = 2.0;
    ct = 1.0;
    mac = mc::getMeanAerodynamicChord( cr, ct );
    EXPECT_NEAR( mac, 1.5556, 1.0e-3 );

    cr = 1.0;
    ct = 2.0;
    mac = mc::getMeanAerodynamicChord( cr, ct );
    EXPECT_NEAR( mac, 1.5556, 1.0e-3 );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMeanAeroChord, CanGetMeanAerodynamicChordFromTable)
{
    std::vector<double> y;
    std::vector<double> c;

    y.push_back( 0.0 );
    c.push_back( 1.0 );

    y.push_back( 1.0 );
    c.push_back( 1.0 );

    double mac = mc::getMeanAerodynamicChord( mc::Table( y, c ) );
    EXPECT_NEAR( mac, 1.0, 1.0e-3 );

    y.clear();
    c.clear();

    y.push_back( 0.0 );
    c.push_back( 2.0 );

    y.push_back( 1.0 );
    c.push_back( 1.0 );

    mac = mc::getMeanAerodynamicChord( mc::Table( y, c ) );
    EXPECT_NEAR( mac, 1.5556, 1.0e-3 );

    y.clear();
    c.clear();

    y.push_back( 0.0 );
    c.push_back( 1.0 );

    y.push_back( 1.0 );
    c.push_back( 2.0 );

    mac = mc::getMeanAerodynamicChord( mc::Table( y, c ) );
    EXPECT_NEAR( mac, 1.5556, 1.0e-3 );

    y.clear();
    c.clear();

    // https://rcplanes.online/cg2_calc.htm

    y.push_back( 0.0 );
    c.push_back( 4.0 );

    y.push_back( 1.0 );
    c.push_back( 2.0 );

    y.push_back( 2.0 );
    c.push_back( 1.0 );

    mac = mc::getMeanAerodynamicChord( mc::Table( y, c ) );
    EXPECT_NEAR( mac, 2.59, 1.0e-2 );
}
