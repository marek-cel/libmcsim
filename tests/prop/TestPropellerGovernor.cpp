#include <gtest/gtest.h>

#include <mcsim/prop/PropellerGovernor.h>

////////////////////////////////////////////////////////////////////////////////

class TestPropellerGovernor : public ::testing::Test
{
protected:

    class PropellerGovernor : public mc::PropellerGovernor
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestPropellerGovernor() {}
    virtual ~TestPropellerGovernor() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropellerGovernor, CanConstruct)
{
    TestPropellerGovernor::PropellerGovernor *gov = nullptr;
    EXPECT_NO_THROW( gov = new TestPropellerGovernor::PropellerGovernor() );
    delete gov;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropellerGovernor, CanDestruct)
{
    TestPropellerGovernor::PropellerGovernor *gov = new TestPropellerGovernor::PropellerGovernor();
    EXPECT_NO_THROW( delete gov );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPropellerGovernor, CanInstantiate)
{
    TestPropellerGovernor::PropellerGovernor gov;
}
