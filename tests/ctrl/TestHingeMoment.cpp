#include <gtest/gtest.h>

#include <mcsim/ctrl/HingeMoment.h>

////////////////////////////////////////////////////////////////////////////////

class TestHingeMoment : public ::testing::Test
{
protected:

    TestHingeMoment() {}
    virtual ~TestHingeMoment() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestHingeMoment, CanConstruct)
{
    mc::HingeMoment *mom = nullptr;
    EXPECT_NO_THROW( mom = new mc::HingeMoment() );
    delete mom;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestHingeMoment, CanDestruct)
{
    mc::HingeMoment *mom = new mc::HingeMoment();
    EXPECT_NO_THROW( delete mom );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestHingeMoment, CanInstantiate)
{
    mc::HingeMoment mom;
}
