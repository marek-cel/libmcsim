#include <gtest/gtest.h>

#include <mcsim/extra/WinchLauncher.h>

////////////////////////////////////////////////////////////////////////////////

class TestWinchLauncher : public ::testing::Test
{
protected:

    class WinchLauncher : public mc::WinchLauncher
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestWinchLauncher() {}
    virtual ~TestWinchLauncher() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWinchLauncher, CanConstruct)
{
    TestWinchLauncher::WinchLauncher *wl = nullptr;
    EXPECT_NO_THROW( wl = new TestWinchLauncher::WinchLauncher() );
    delete wl;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWinchLauncher, CanDestruct)
{
    TestWinchLauncher::WinchLauncher *wl = new TestWinchLauncher::WinchLauncher();
    EXPECT_NO_THROW( delete wl );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWinchLauncher, CanInstantiate)
{
    TestWinchLauncher::WinchLauncher wl;
}
