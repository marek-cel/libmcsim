#include <gtest/gtest.h>

#include <mcsim/aero/StabilizerVer.h>

////////////////////////////////////////////////////////////////////////////////

class TestStabilizerVer : public ::testing::Test
{
protected:

    class StabilizerVer : public mc::StabilizerVer
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestStabilizerVer() {}
    virtual ~TestStabilizerVer() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerVer, CanConstruct)
{
    TestStabilizerVer::StabilizerVer *stabVer = nullptr;
    EXPECT_NO_THROW( stabVer = new TestStabilizerVer::StabilizerVer() );
    delete stabVer;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerVer, CanDestruct)
{
    TestStabilizerVer::StabilizerVer *stabVer = new TestStabilizerVer::StabilizerVer();
    EXPECT_NO_THROW( delete stabVer );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestStabilizerVer, CanInstantiate)
{
    TestStabilizerVer::StabilizerVer stabVer;
}
