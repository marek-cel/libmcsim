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

TEST_F(TestSchrenkDist, DISABLED_CanCalculateEllipsoidWing1)
{
    mc::SchrenkDist dist;
    std::vector span {0.0, 1.0};
    std::vector chord {1.0, 1.0};

    dist.set_area(2.0);
    dist.set_span(2.0);
    dist.set_chord(mc::Table(span, chord));

    double y = 0.0;
    while ( y < 1.0 )
    {
        double c = dist.GetEllipsoidWingChord(y);
        std::cout << y << " " << c << std::endl;
        y += 0.1;
    }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, CanCalculateEllipsoidWing)
{
    mc::SchrenkDist dist;
    std::vector span {0.0, 1.0};
    std::vector chord {1.0, 0.5};

    dist.set_area(1.5);
    dist.set_span(2.0);
    dist.set_chord(mc::Table(span, chord));

    double y = 0.0;
    while ( y < 1.0 )
    {
        double c = dist.GetEllipsoidWingChord(y);
        std::cout << y << " " << c << std::endl;
        y += 0.1;
    }
    exit(0);
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, DISABLED_CanCalculateRectangularWing)
{
    std::vector<double> y_ref;
    std::vector<double> cl_ref;

    // expected values Raymer DP. Aircraft Design: A Conceptual Approach, 6th Edition, 2018, p.84
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
        double tol = 0.01;
        if ( y > 0.3 ) tol = 0.05;
        if ( y > 0.7 ) tol = 0.1;
        std::cout << y << " " << cl << " " << cl_ref.at(i) << std::endl;
        EXPECT_NEAR(cl, cl_ref.at(i), tol) << "Mismatch at span= " << y;
    }
}

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSchrenkDist, DISABLED_CanCalculateTrapezoidalWing)
{
    std::vector<double> y_ref;
    std::vector<double> cl_ref;

    // expected values Raymer DP. Aircraft Design: A Conceptual Approach, 6th Edition, 2018, p.84
    CsvFileReader::ReadData("../tests/aero/data/schrenk_dist_taper.csv", &y_ref, &cl_ref);

    EXPECT_GT(y_ref.size(), 0) << "No reference data.";
    EXPECT_GT(cl_ref.size(), 0) << "No reference data.";
    EXPECT_EQ(y_ref.size(), cl_ref.size()) << "Reference data corrupted.";

    mc::SchrenkDist dist;
    std::vector span {0.0, 1.0};
    std::vector chord {1.0, 0.5};
    mc::Table xxx(span, chord);

    dist.set_area(2.0);
    dist.set_span(2.0);
    dist.set_chord(xxx);

    for ( unsigned int i = 0; i < y_ref.size(); i++ )
    {
        double y = y_ref.at(i);
        double cl = dist.GetLiftCoefDist(y);
        double tol = 0.01;
        if ( y > 0.3 ) tol = 0.05;
        if ( y > 0.7 ) tol = 0.1;
        //std::cout << y << " " << xxx.GetValue(y) << std::endl;
        std::cout << y << " " << cl << " " << cl_ref.at(i) << std::endl;
        //EXPECT_NEAR(cl, cl_ref.at(i), tol) << "Mismatch at span= " << y;
    }
    exit(0);
}
