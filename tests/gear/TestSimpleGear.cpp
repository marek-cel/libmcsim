#include <gtest/gtest.h>

#include <mcsim/gear/SimpleGear.h>

////////////////////////////////////////////////////////////////////////////////

class TestSimpleGear : public ::testing::Test
{
protected:

    TestSimpleGear() {}
    virtual ~TestSimpleGear() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleGear, CanConstruct)
{
    mc::SimpleGear *sg = nullptr;
    EXPECT_NO_THROW( sg = new mc::SimpleGear() );
    delete sg;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleGear, CanDestruct)
{
    mc::SimpleGear *sg = new mc::SimpleGear();
    EXPECT_NO_THROW( delete sg );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleGear, CanInstantiate)
{
    mc::SimpleGear sg;
}
