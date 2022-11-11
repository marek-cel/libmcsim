#include <gtest/gtest.h>

////////////////////////////////////////////////////////////////////////////////

class TestDummy : public ::testing::Test
{
protected:

    TestAtmosphereUS76() {}
    virtual ~TestAtmosphereUS76() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestDummy, CanAssertTrue)
{
    ASSERT_TRUE( true );
}
