#include <gtest/gtest.h>

#include <mcsim/rotor/TailRotor.h>

////////////////////////////////////////////////////////////////////////////////

class TestTailRotor : public ::testing::Test
{
protected:

    TestTailRotor() {}
    virtual ~TestTailRotor() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestTailRotor, CanConstruct)
{
    mc::TailRotor *rotor = nullptr;
    EXPECT_NO_THROW( rotor = new mc::TailRotor() );
    delete rotor;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestTailRotor, CanDestruct)
{
    mc::TailRotor *rotor = new mc::TailRotor();
    EXPECT_NO_THROW( delete rotor );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestTailRotor, CanInstantiate)
{
    mc::TailRotor rotor;
}
