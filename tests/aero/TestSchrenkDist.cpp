#include <gtest/gtest.h>

#include <mcsim/aero/SchrenkDist.h>

////////////////////////////////////////////////////////////////////////////////

class TestSchrenkDist : public ::testing::Test
{
protected:

    TestSchrenkDist() {}
    virtual ~TestSchrenkDist() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanConstruct)
{
    mc::SchrenkDist *dist = nullptr;
    EXPECT_NO_THROW( dist = new mc::SchrenkDist() );
    delete dist;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanDestruct)
{
    mc::SchrenkDist *dist = new mc::SchrenkDist();
    EXPECT_NO_THROW( delete dist );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanInstantiate)
{
    mc::SchrenkDist dist;
}
