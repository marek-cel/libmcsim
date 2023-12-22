#include <gtest/gtest.h>

#include <mcsim/gear/Pacejka.h>

////////////////////////////////////////////////////////////////////////////////

class TestPacejka : public ::testing::Test
{
protected:

    TestPacejka() {}
    virtual ~TestPacejka() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPacejka, CanComputePacejkaFormula)
{
    // TODO
}
