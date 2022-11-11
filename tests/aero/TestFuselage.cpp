#include <gtest/gtest.h>

#include <mcsim/aero/Fuselage.h>

////////////////////////////////////////////////////////////////////////////////

class TestFuselage : public ::testing::Test
{
protected:

    TestFuselage() {}
    virtual ~TestFuselage() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestFuselage, CanConstruct)
{
    mc::Fuselage *fuselage = nullptr;
    EXPECT_NO_THROW( fuselage = new mc::Fuselage() );
    delete fuselage;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestFuselage, CanDestruct)
{
    mc::Fuselage *fuselage = new mc::Fuselage();
    EXPECT_NO_THROW( delete fuselage );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestFuselage, CanInstantiate)
{
    mc::Fuselage fuselage;
}
