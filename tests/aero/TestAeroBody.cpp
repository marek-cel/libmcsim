#include <gtest/gtest.h>

#include <mcsim/aero/AeroBody.h>

////////////////////////////////////////////////////////////////////////////////

class TestAeroBody : public ::testing::Test
{
protected:

    TestAeroBody() {}
    virtual ~TestAeroBody() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroBody, CanConstruct)
{
    mc::AeroBody* fuselage = nullptr;
    EXPECT_NO_THROW( fuselage = new mc::AeroBody );
    delete fuselage;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroBody, CanDestruct)
{
    mc::AeroBody* fuselage = new mc::AeroBody();
    EXPECT_NO_THROW( delete fuselage );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroBody, CanInstantiate)
{
    mc::AeroBody fuselage;
}
