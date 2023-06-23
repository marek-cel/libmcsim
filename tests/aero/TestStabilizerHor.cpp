#include <gtest/gtest.h>

#include <mcsim/aero/StabilizerHor.h>

////////////////////////////////////////////////////////////////////////////////

class TestStabilizerHor : public ::testing::Test
{
protected:

    class StabilizerHor : public mc::StabilizerHor
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestStabilizerHor() {}
    virtual ~TestStabilizerHor() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerHor, CanConstruct)
{
    TestStabilizerHor::StabilizerHor *stabHor = nullptr;
    EXPECT_NO_THROW( stabHor = new TestStabilizerHor::StabilizerHor() );
    delete stabHor;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerHor, CanDestruct)
{
    TestStabilizerHor::StabilizerHor *stabHor = new TestStabilizerHor::StabilizerHor();
    EXPECT_NO_THROW( delete stabHor );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerHor, CanInstantiate)
{
    TestStabilizerHor::StabilizerHor stabHor;
}
