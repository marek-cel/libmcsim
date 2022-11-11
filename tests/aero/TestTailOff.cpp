#include <gtest/gtest.h>

#include <mcsim/aero/TailOff.h>

////////////////////////////////////////////////////////////////////////////////

class TestTailOff : public ::testing::Test
{
protected:

    TestTailOff() {}
    virtual ~TestTailOff() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestTailOff, CanConstruct)
{
    mc::TailOff *tailOff = nullptr;
    EXPECT_NO_THROW( tailOff = new mc::TailOff() );
    delete tailOff;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestTailOff, CanDestruct)
{
    mc::TailOff *tailOff = new mc::TailOff();
    EXPECT_NO_THROW( delete tailOff );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestTailOff, CanInstantiate)
{
    mc::TailOff tailOff;
}
