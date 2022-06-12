#include <gtest/gtest.h>

#include <mcsim/utils/AeroAngles.h>

////////////////////////////////////////////////////////////////////////////////

class TestAeroAngles : public ::testing::Test
{
protected:
    TestAeroAngles() {}
    virtual ~TestAeroAngles() {}
    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroAngles, CanTrue)
{
    EXPECT_TRUE( false );
}
