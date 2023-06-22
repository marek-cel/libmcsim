#include <gtest/gtest.h>

#include <mcsim/aero/WingBody.h>

////////////////////////////////////////////////////////////////////////////////

class TestWingBody : public ::testing::Test
{
protected:

    TestWingBody() {}
    virtual ~TestWingBody() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingBody, CanConstruct)
{
    mc::WingBody *wb = nullptr;
    EXPECT_NO_THROW( wb = new mc::WingBody() );
    delete wb;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingBody, CanDestruct)
{
    mc::WingBody *wb = new mc::WingBody();
    EXPECT_NO_THROW( delete wb );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingBody, CanInstantiate)
{
    mc::WingBody wb;
}
