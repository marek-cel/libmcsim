#include <gtest/gtest.h>

#include <mcsim/prop/PistonEngine.h>

////////////////////////////////////////////////////////////////////////////////

class TestPistonEngine : public ::testing::Test
{
protected:

    TestPistonEngine() {}
    virtual ~TestPistonEngine() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPistonEngine, CanConstruct)
{
    mc::PistonEngine *engine = nullptr;
    EXPECT_NO_THROW( engine = new mc::PistonEngine() );
    delete engine;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPistonEngine, CanDestruct)
{
    mc::PistonEngine *engine = new mc::PistonEngine();
    EXPECT_NO_THROW( delete engine );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestPistonEngine, CanInstantiate)
{
    mc::PistonEngine engine;
}
