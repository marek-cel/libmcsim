#include <gtest/gtest.h>

#include <mcsim/extra/WingRunner.h>

////////////////////////////////////////////////////////////////////////////////

class TestWingRunner : public ::testing::Test
{
protected:

    class WingRunner : public mc::WingRunner
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestWingRunner() {}
    virtual ~TestWingRunner() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingRunner, CanConstruct)
{
    TestWingRunner::WingRunner *wingRunner = nullptr;
    EXPECT_NO_THROW( wingRunner = new TestWingRunner::WingRunner() );
    delete wingRunner;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingRunner, CanDestruct)
{
    TestWingRunner::WingRunner *wingRunner = new TestWingRunner::WingRunner();
    EXPECT_NO_THROW( delete wingRunner );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingRunner, CanInstantiate)
{
    TestWingRunner::WingRunner wingRunner;
}
