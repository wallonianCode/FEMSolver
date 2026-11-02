#include <gtest/gtest.h>

#include "../include/fem/solvers/GlobalDisplacementsReconstructor.hpp"



TEST(GlobalDisplacementsReconstructorTests, NoFreeDisplacements)
{
    Eigen::VectorXd expected_vector(3);
    const Eigen::VectorXd free_displacements;
    const DofPartition partition 
    {
        {}, //free displacements vector empty
        {
         {0, 0.0},
         {1, 0.0},
         {2, 1.0}
        }
    };


    const Eigen::VectorXd global_displacement_vector =
        GlobalDisplacementsReconstructor::get_global_displacements_vector(
            free_displacements,
            partition
        );

    expected_vector << 0.0, 0.0, 1.0;
    EXPECT_TRUE(global_displacement_vector.isApprox(expected_vector, 1e-12));
}


TEST(GlobalDisplacementsReconstructorTests, NoConstrainedDisplacements)
{
    Eigen::VectorXd free_displacements(3);
    free_displacements << 1.0, 2.0, 3.0;

    Eigen::VectorXd expected_vector(3);
    expected_vector << 1.0, 2.0, 3.0;

    const DofPartition partition
    {
        {0, 1, 2},
        {}
    };

    const Eigen::VectorXd global_displacement_vector =
        GlobalDisplacementsReconstructor::get_global_displacements_vector(
            free_displacements, partition);

    EXPECT_TRUE(global_displacement_vector.isApprox(expected_vector));
}


TEST(GlobalDisplacementsReconstructorTests, FreeAndConstrainedDisplacements)
{
    Eigen::VectorXd free_displacements(2);
    free_displacements << 1.0, 3.0;

    Eigen::VectorXd expected_vector(3);
    expected_vector << 1.0, 0.5, 3.0;

    const DofPartition partition
    {
        {0, 2},
        {
            {1, 0.5}
        }
    };

    const Eigen::VectorXd global_displacement_vector =
        GlobalDisplacementsReconstructor::get_global_displacements_vector(
            free_displacements, partition);

    EXPECT_TRUE(global_displacement_vector.isApprox(expected_vector));
}


//test for mismatch between partition and free disp argument (unequal sizes)
TEST(GlobalDisplacementsReconstructorTests, ArgumentsMismatch)
{
    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    const DofPartition partition
    {
        {0, 2},
        {
            {1, 0.5}
        }
    };

    EXPECT_THROW(
        GlobalDisplacementsReconstructor::get_global_displacements_vector(
            free_displacements, partition),
        std::invalid_argument);
}


