#include <gtest/gtest.h>

#include <mcsim/prop/Propeller.h>

class TestPropeller : public ::testing::Test
{
protected:

    class Propeller : public mc::Propeller
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestPropeller() {}
    virtual ~TestPropeller() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestPropeller, CanConstruct)
{
    TestPropeller::Propeller* prop = nullptr;
    EXPECT_NO_THROW(prop = new TestPropeller::Propeller());
    delete prop;
}

TEST_F(TestPropeller, CanDestruct)
{
    TestPropeller::Propeller* prop = new TestPropeller::Propeller();
    EXPECT_NO_THROW(delete prop);
}

TEST_F(TestPropeller, CanInstantiate)
{
    TestPropeller::Propeller prop;
}
