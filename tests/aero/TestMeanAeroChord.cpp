#include <gtest/gtest.h>

#include <mcsim/aero/MeanAeroChord.h>

////////////////////////////////////////////////////////////////////////////////

class TestMeanAeroChord : public ::testing::Test
{
protected:

    TestMeanAeroChord() {}
    virtual ~TestMeanAeroChord() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestMeanAeroChord, CanGetMeanAerodynamicChord)
{
    ASSERT_TRUE( true );
}
