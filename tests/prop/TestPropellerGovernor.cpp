#include <gtest/gtest.h>

#include <mcsim/prop/PropellerGovernor.h>

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

TEST_F(TestPropellerGovernor, CanInstantiate)
{
    // TODO
}
