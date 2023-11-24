#include <gtest/gtest.h>

#include <mcsim/aero/WingBody.h>

////////////////////////////////////////////////////////////////////////////////

class TestWingBody : public ::testing::Test
{
protected:

    class WingBody : public mc::WingBody
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestWingBody() {}
    virtual ~TestWingBody() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingBody, CanConstruct)
{
    TestWingBody::WingBody *wb = nullptr;
    EXPECT_NO_THROW( wb = new TestWingBody::WingBody() );
    delete wb;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingBody, CanDestruct)
{
    TestWingBody::WingBody *wb = new TestWingBody::WingBody();
    EXPECT_NO_THROW( delete wb );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestWingBody, CanInstantiate)
{
    TestWingBody::WingBody wb;
}
