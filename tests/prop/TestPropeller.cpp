#include <gtest/gtest.h>

#include <mcsim/prop/Propeller.h>

////////////////////////////////////////////////////////////////////////////////

class TestPropeller : public ::testing::Test
{
protected:

    TestPropeller() {}
    virtual ~TestPropeller() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropeller, CanConstruct)
{
    mc::Propeller *prop = nullptr;
    EXPECT_NO_THROW( prop = new mc::Propeller() );
    delete prop;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropeller, CanDestruct)
{
    mc::Propeller *prop = new mc::Propeller();
    EXPECT_NO_THROW( delete prop );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropeller, CanInstantiate)
{
    mc::Propeller prop;
}
