#include <gtest/gtest.h>

#include <mcsim/mass/InertiaMatrix.h>

class TestInertiaMatrix : public ::testing::Test
{
protected:

    TestInertiaMatrix() {}
    virtual ~TestInertiaMatrix() {}

    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(TestInertiaMatrix, CanGetInertiaMatrix)
{
    const double mass = 100.0;
    const mc::Vector3 s_bas(1.0, 2.0, 3.0);
    const mc::Matrix3x3 i_bas(11.0, 12.0, 13.0,
                              12.0, 22.0, 23.0,
                              13.0, 23.0, 33.0);

    mc::Matrix6x6 im = mc::GetInertiaMatrix(mass, s_bas, i_bas);

    // Fossen T.: Handbook of Marine Craft Hydrodynamics and Motion Control. 2011, p.52
    // Maryniak J.: Ogolny model matematyczny sterowanego samolotu. in Mechanika w Lotnictwie, 1992, p.577
    //
    // |  m   0   0   0    Sz  -Sy  |
    // |  0   m   0  -Sz   0    Sx  |
    // |  0   0   m   Sy  -Sx   0   |
    // |  0  -Sz  Sy  Ix   Ixy  Ixz |
    // |  Sz  0  -Sx  Ixy  Iy   Iyz |
    // | -Sy  Sx  0   Ixz  Iyz  Iz  |

    EXPECT_NEAR( im(0,0),  mass       , 1.0e-6 );
    EXPECT_NEAR( im(0,1),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(0,2),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(0,3),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(0,4),  s_bas.z()  , 1.0e-6 );
    EXPECT_NEAR( im(0,5), -s_bas.y()  , 1.0e-6 );

    EXPECT_NEAR( im(1,0),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(1,1),  mass       , 1.0e-6 );
    EXPECT_NEAR( im(1,2),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(1,3), -s_bas.z()  , 1.0e-6 );
    EXPECT_NEAR( im(1,4),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(1,5),  s_bas.x()  , 1.0e-6 );

    EXPECT_NEAR( im(2,0),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(2,1),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(2,2),  mass       , 1.0e-6 );
    EXPECT_NEAR( im(2,3),  s_bas.y()  , 1.0e-6 );
    EXPECT_NEAR( im(2,4), -s_bas.x()  , 1.0e-6 );
    EXPECT_NEAR( im(2,5),  0.0        , 1.0e-6 );

    EXPECT_NEAR( im(3,0),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(3,1), -s_bas.z()  , 1.0e-6 );
    EXPECT_NEAR( im(3,2),  s_bas.y()  , 1.0e-6 );
    EXPECT_NEAR( im(3,3),  i_bas(0,0) , 1.0e-6 );
    EXPECT_NEAR( im(3,4),  i_bas(0,1) , 1.0e-6 );
    EXPECT_NEAR( im(3,5),  i_bas(0,2) , 1.0e-6 );

    EXPECT_NEAR( im(4,0),  s_bas.z()  , 1.0e-6 );
    EXPECT_NEAR( im(4,1),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(4,2), -s_bas.x()  , 1.0e-6 );
    EXPECT_NEAR( im(4,3),  i_bas(1,0) , 1.0e-6 );
    EXPECT_NEAR( im(4,4),  i_bas(1,1) , 1.0e-6 );
    EXPECT_NEAR( im(4,5),  i_bas(1,2) , 1.0e-6 );

    EXPECT_NEAR( im(5,0), -s_bas.y()  , 1.0e-6 );
    EXPECT_NEAR( im(5,1),  s_bas.x()  , 1.0e-6 );
    EXPECT_NEAR( im(5,2),  0.0        , 1.0e-6 );
    EXPECT_NEAR( im(5,3),  i_bas(2,0) , 1.0e-6 );
    EXPECT_NEAR( im(5,4),  i_bas(2,1) , 1.0e-6 );
    EXPECT_NEAR( im(5,5),  i_bas(2,2) , 1.0e-6 );
}
