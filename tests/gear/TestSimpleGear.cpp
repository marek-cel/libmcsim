#include <gtest/gtest.h>

#include <mcsim/gear/SimpleGear.h>

class TestSimpleGear : public ::testing::Test
{
protected:

    class SimpleGear : public mc::SimpleGear
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestSimpleGear() {}
    virtual ~TestSimpleGear() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestSimpleGear, CanInstantiate)
{
    // TODO
}
