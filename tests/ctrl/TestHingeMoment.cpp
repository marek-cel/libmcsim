#include <gtest/gtest.h>

#include <mcsim/ctrl/HingeMoment.h>

class TestHingeMoment : public ::testing::Test
{
protected:

    class HingeMoment : public mc::HingeMoment
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestHingeMoment() {}
    virtual ~TestHingeMoment() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestHingeMoment, CanInstantiate)
{
    TestHingeMoment::HingeMoment mom;
}

TEST_F(TestHingeMoment, CanCompute)
{
    // TODO
}
