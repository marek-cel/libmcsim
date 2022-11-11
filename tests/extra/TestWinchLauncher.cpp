#include <gtest/gtest.h>

#include <mcsim/extra/WinchLauncher.h>

////////////////////////////////////////////////////////////////////////////////

class TestWinchLauncher : public ::testing::Test
{
protected:

    TestWinchLauncher() {}
    virtual ~TestWinchLauncher() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWinchLauncher, CanConstruct)
{
    mc::WinchLauncher *winchLauncher = nullptr;
    EXPECT_NO_THROW( winchLauncher = new mc::WinchLauncher() );
    delete winchLauncher;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWinchLauncher, CanDestruct)
{
    mc::WinchLauncher *winchLauncher = new mc::WinchLauncher();
    EXPECT_NO_THROW( delete winchLauncher );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWinchLauncher, CanInstantiate)
{
    mc::WinchLauncher winchLauncher;
}
