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

TEST_F(TestPointMass, CanConstruct)
{
    TestPointMass::PointMass* mass = nullptr;
    EXPECT_NO_THROW(mass = new TestPointMass::PointMass());
    delete mass;
}

TEST_F(TestPointMass, CanDestruct)
{
    TestPointMass::PointMass* mass = new TestPointMass::PointMass();
    EXPECT_NO_THROW(delete mass);
}

TEST_F(TestPointMass, CanInstantiate)
{
    TestPointMass::PointMass mass;
}
