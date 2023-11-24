#include <gtest/gtest.h>

#include <mcsim/aero/Fuselage.h>

////////////////////////////////////////////////////////////////////////////////

class TestFuselage : public ::testing::Test
{
protected:

    class Fuselage : public mc::Fuselage
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestFuselage() {}
    virtual ~TestFuselage() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestFuselage, CanConstruct)
{
    TestFuselage::Fuselage* fuselage = nullptr;
    EXPECT_NO_THROW( fuselage = new TestFuselage::Fuselage );
    delete fuselage;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestFuselage, CanDestruct)
{
    TestFuselage::Fuselage* fuselage = new TestFuselage::Fuselage();
    EXPECT_NO_THROW( delete fuselage );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestFuselage, CanInstantiate)
{
    TestFuselage::Fuselage fuselage;
}
