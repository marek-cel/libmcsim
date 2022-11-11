#include <gtest/gtest.h>

#include <mcsim/mass/PointMass.h>

////////////////////////////////////////////////////////////////////////////////

class TestPointMass : public ::testing::Test
{
protected:

    TestPointMass() {}
    virtual ~TestPointMass() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPointMass, CanConstruct)
{
    mc::PointMass *mass = nullptr;
    EXPECT_NO_THROW( mass = new mc::PointMass() );
    delete mass;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPointMass, CanDestruct)
{
    mc::PointMass *mass = new mc::PointMass();
    EXPECT_NO_THROW( delete mass );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPointMass, CanInstantiate)
{
    mc::PointMass mass;
}
