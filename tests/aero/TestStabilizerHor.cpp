#include <gtest/gtest.h>

#include <mcsim/aero/StabilizerHor.h>

////////////////////////////////////////////////////////////////////////////////

class TestStabilizerHor : public ::testing::Test
{
protected:

    TestStabilizerHor() {}
    virtual ~TestStabilizerHor() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerHor, CanConstruct)
{
    mc::StabilizerHor *stabHor = nullptr;
    EXPECT_NO_THROW( stabHor = new mc::StabilizerHor() );
    delete stabHor;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerHor, CanDestruct)
{
    mc::StabilizerHor *stabHor = new mc::StabilizerHor();
    EXPECT_NO_THROW( delete stabHor );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerHor, CanInstantiate)
{
    mc::StabilizerHor stabHor;
}
