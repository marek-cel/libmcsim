#include <gtest/gtest.h>

#include <mcsim/extra/WingRunner.h>

////////////////////////////////////////////////////////////////////////////////

class TestWingRunner : public ::testing::Test
{
protected:

    TestWingRunner() {}
    virtual ~TestWingRunner() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingRunner, CanConstruct)
{
    mc::WingRunner *wingRunner = nullptr;
    EXPECT_NO_THROW( wingRunner = new mc::WingRunner() );
    delete wingRunner;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingRunner, CanDestruct)
{
    mc::WingRunner *wingRunner = new mc::WingRunner();
    EXPECT_NO_THROW( delete wingRunner );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingRunner, CanInstantiate)
{
    mc::WingRunner wingRunner;
}
