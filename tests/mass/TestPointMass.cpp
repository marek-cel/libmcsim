#include <gtest/gtest.h>

#include <mcsim/mass/PointMass.h>

class TestPointMass : public ::testing::Test
{
protected:

    class PointMass : public mc::PointMass
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestPointMass() {}
    virtual ~TestPointMass() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestPointMass, CanInstantiate)
{
    // TODO
}
