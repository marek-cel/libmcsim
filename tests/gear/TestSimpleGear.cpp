#include <gtest/gtest.h>

#include <mcsim/gear/SimpleGear.h>

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleGear, CanConstruct)
{
    TestSimpleGear::SimpleGear *sg = nullptr;
    EXPECT_NO_THROW( sg = new TestSimpleGear::SimpleGear() );
    delete sg;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleGear, CanDestruct)
{
    TestSimpleGear::SimpleGear *sg = new TestSimpleGear::SimpleGear();
    EXPECT_NO_THROW( delete sg );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSimpleGear, CanInstantiate)
{
    TestSimpleGear::SimpleGear sg;
}
