#include <gtest/gtest.h>

#include <mcsim/prop/PistonEngine.h>

class TestPistonEngine : public ::testing::Test
{
protected:

    class PistonEngine : public mc::PistonEngine
    {
    public:
        const Data& data() const override { return data_; }

    private:
        Data data_;
    };

    TestPistonEngine() {}
    virtual ~TestPistonEngine() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestPistonEngine, CanConstruct)
{
    TestPistonEngine::PistonEngine* engine = nullptr;
    EXPECT_NO_THROW(engine = new TestPistonEngine::PistonEngine());
    delete engine;
}

TEST_F(TestPistonEngine, CanDestruct)
{
    TestPistonEngine::PistonEngine* engine = new TestPistonEngine::PistonEngine();
    EXPECT_NO_THROW(delete engine);
}

TEST_F(TestPistonEngine, CanInstantiate)
{
    TestPistonEngine::PistonEngine engine;
}
