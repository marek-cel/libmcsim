#include <gtest/gtest.h>

#include <mcsim/aero/StabilizerVer.h>

////////////////////////////////////////////////////////////////////////////////

class TestStabilizerVer : public ::testing::Test
{
protected:

    TestStabilizerVer() {}
    virtual ~TestStabilizerVer() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerVer, CanConstruct)
{
    mc::StabilizerVer *stabVer = nullptr;
    EXPECT_NO_THROW( stabVer = new mc::StabilizerVer() );
    delete stabVer;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerVer, CanDestruct)
{
    mc::StabilizerVer *stabVer = new mc::StabilizerVer();
    EXPECT_NO_THROW( delete stabVer );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerVer, CanInstantiate)
{
    mc::StabilizerVer stabVer;
}
