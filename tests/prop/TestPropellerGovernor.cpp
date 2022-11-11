#include <gtest/gtest.h>

#include <mcsim/prop/PropellerGovernor.h>

////////////////////////////////////////////////////////////////////////////////

class TestPropellerGovernor : public ::testing::Test
{
protected:

    TestPropellerGovernor() {}
    virtual ~TestPropellerGovernor() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropellerGovernor, CanConstruct)
{
    mc::PropellerGovernor *gov = nullptr;
    EXPECT_NO_THROW( gov = new mc::PropellerGovernor() );
    delete gov;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropellerGovernor, CanDestruct)
{
    mc::PropellerGovernor *gov = new mc::PropellerGovernor();
    EXPECT_NO_THROW( delete gov );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropellerGovernor, CanInstantiate)
{
    mc::PropellerGovernor gov;
}
