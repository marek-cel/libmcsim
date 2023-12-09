#include <gtest/gtest.h>

#include <mcsim/aero/SchrenkDist.h>

#include <CsvFileReader.h>

////////////////////////////////////////////////////////////////////////////////

class TestSchrenkDist : public ::testing::Test
{
protected:

    TestSchrenkDist() {}
    virtual ~TestSchrenkDist() {}

    void SetUp() override {}
    void TearDown() override {}
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanConstruct)
{
    mc::SchrenkDist *dist = nullptr;
    EXPECT_NO_THROW( dist = new mc::SchrenkDist() );
    delete dist;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanDestruct)
{
    mc::SchrenkDist *dist = new mc::SchrenkDist();
    EXPECT_NO_THROW( delete dist );
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanInstantiate)
{
    mc::SchrenkDist dist;
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanCalculateRectangularWing)
{
    std::vector<double> y_ref;
    std::vector<double> cl_ref;

    // expected values 
    // Raymer DP. Aircraft Design: A Conceptual Approach, 6th Edition, 2018, p.84
    CsvFileReader::ReadData("../tests/aero/data/schrenk_dist_rect.csv", &y_ref, &cl_ref);

    EXPECT_GT(y_ref.size(), 0) << "No reference data.";
    EXPECT_GT(cl_ref.size(), 0) << "No reference data.";
    EXPECT_EQ(y_ref.size(), cl_ref.size()) << "Reference data corrupted.";

    mc::SchrenkDist dist;
    std::vector span {0.0, 1.0};
    std::vector chord {1.0, 1.0};

    dist.set_area(2.0);
    dist.set_span(2.0);
    dist.set_chord(mc::Table(span, chord));

    for ( unsigned int i = 0; i < y_ref.size(); i++ )
    {
        double y = y_ref.at(i);
        double cl = dist.GetLiftCoefDist(y);
        double tol = 0.05;
        EXPECT_NEAR(cl, cl_ref.at(i), tol) << "Mismatch at span= " << y;
    }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanCalculateTrapezoidalWing)
{
    std::vector<double> y_ref;
    std::vector<double> cl_ref;

    // expected values 
    // Schrenk O.: A Simple Approximation Method for Obtaining the Spanwise Lift Distribution, NACA TM-948, Figure 6
    CsvFileReader::ReadData("../tests/aero/data/schrenk_dist_taper.csv", &y_ref, &cl_ref);

    EXPECT_GT(y_ref.size(), 0) << "No reference data.";
    EXPECT_GT(cl_ref.size(), 0) << "No reference data.";
    EXPECT_EQ(y_ref.size(), cl_ref.size()) << "Reference data corrupted.";

    mc::SchrenkDist dist;
    std::vector span {0.0, 1.0};
    std::vector chord {1.0, 0.5};
    mc::Table xxx(span, chord);

    dist.set_area(1.5);
    dist.set_span(2.0);
    dist.set_chord(xxx);

    for ( unsigned int i = 0; i < y_ref.size(); i++ )
    {
        double y = y_ref.at(i);
        double cl = dist.GetLiftCoefDist(y);
        double tol = 0.05 * cl_ref.at(i);
        EXPECT_NEAR(cl, cl_ref.at(i), tol) << "Mismatch at span= " << y;
    }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanCalculateDeltaWing)
{
    std::vector<double> y_ref;
    std::vector<double> l_ref;

    // expected values 
    // Corke TC.: Design of Aircraft, 2003, p.240
    CsvFileReader::ReadData("../tests/aero/data/schrenk_dist_delta.csv", &y_ref, &l_ref);

    EXPECT_GT(y_ref.size(), 0) << "No reference data.";
    EXPECT_GT(l_ref.size(), 0) << "No reference data.";
    EXPECT_EQ(y_ref.size(), l_ref.size()) << "Reference data corrupted.";

    double luf = 88817.0 / 2.0;

    mc::SchrenkDist dist;
    std::vector span {0.0, 16.1};
    std::vector chord {32.2, 0.0};

    dist.set_area(519.0);
    dist.set_span(32.2);
    dist.set_chord(mc::Table(span, chord));

    for ( unsigned int i = 0; i < y_ref.size(); i++ )
    {
        double y = y_ref.at(i);
        double l = luf * dist.GetLiftCoefDist(y);
        double tol = 0.05 * l_ref.at(i);
        EXPECT_NEAR(l, l_ref.at(i), tol) << "Mismatch at span= " << y;
    }
}
