#include <gtest/gtest.h>

#include <mcsim/gear/SimpleSupport.h>

////////////////////////////////////////////////////////////////////////////////

class TestSimpleSupport : public ::testing::Test
{
protected:

    TestSimpleSupport() {}
    virtual ~TestSimpleSupport() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleSupport, CanConstruct)
{
    mc::SimpleSupport *simpleSupport = nullptr;
    EXPECT_NO_THROW( simpleSupport = new mc::SimpleSupport() );
    delete simpleSupport;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleSupport, CanDestruct)
{
    mc::SimpleSupport *simpleSupport = new mc::SimpleSupport();
    EXPECT_NO_THROW( delete simpleSupport );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleSupport, CanInstantiate)
{
    mc::SimpleSupport simpleSupport;
}
