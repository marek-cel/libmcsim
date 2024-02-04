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

TEST_F(TestPistonEngine, CanInstantiate)
{
    // TODO
}
