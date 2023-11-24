#include <gtest/gtest.h>

#include <mcsim/aero/AeroBody.h>

////////////////////////////////////////////////////////////////////////////////

class TestAeroBody : public ::testing::Test
{
protected:

    class AeroBody : public mc::AeroBody
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestAeroBody() {}
    virtual ~TestAeroBody() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroBody, CanConstruct)
{
    TestAeroBody::AeroBody* ab = nullptr;
    EXPECT_NO_THROW( ab = new TestAeroBody::AeroBody );
    delete ab;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroBody, CanDestruct)
{
    TestAeroBody::AeroBody* ab = new TestAeroBody::AeroBody();
    EXPECT_NO_THROW( delete ab );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestAeroBody, CanInstantiate)
{
    TestAeroBody::AeroBody ab;
}
