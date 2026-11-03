#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../../include/fem/solvers/LinearSystemSolver.hpp"

//class responsability: solving a reduced system, independantly of the 
//way the system in question was built. 

//returns zero displacement
TEST(LinearSystemSolverTests, ZeroForcesVector)
{
    Eigen::Matrix2d stiffness_matrix;
    stiffness_matrix << 4.0, 2.0,
                        2.0, 3.0;

    const Eigen::Vector2d force_vector =
        Eigen::Vector2d::Zero();

    const ReducedSystem reduced_system
    {
        stiffness_matrix,
        force_vector,
        {}  // the partition does not matter for the solving
            //the force_vector already contains the right ordering
    };

    const Eigen::Vector2d expected_displacements =
        Eigen::Vector2d::Zero();

    const Eigen::VectorXd displacements =
        LinearSystemSolver::solve_reduced_system(reduced_system);

    EXPECT_TRUE(displacements.isApprox(expected_displacements));

}


//correctly solves a known system.
TEST(LinearSystemSolverTests, NonZeroForcesVector)
{
    Eigen::Matrix2d stiffness_matrix;
    stiffness_matrix << 4.0, 2.0,
                        2.0, 3.0;

    const Eigen::Vector2d force_vector = {8.0, 8.0};

    const ReducedSystem reduced_system
    {
        stiffness_matrix,
        force_vector,
        {}  // The solver only operates on the reduced system.
            // DOF partition information is not required for solving.
    };

    const Eigen::Vector2d expected_displacements = {1.0, 2.0};

    const Eigen::VectorXd displacements =
        LinearSystemSolver::solve_reduced_system(reduced_system);

    EXPECT_TRUE(displacements.isApprox(expected_displacements));
}


//symmetric positive definite matrix
//correctness accross different systems
TEST(LinearSystemSolverTests, DifferentSPDStiffnessMatrices)
{
    // System 1
    Eigen::Matrix2d stiffness_matrix_1;
    stiffness_matrix_1 << 4.0, 2.0,
                          2.0, 3.0;

    Eigen::Vector2d force_vector_1;
    force_vector_1 << 8.0, 8.0;

    const ReducedSystem reduced_system_1
    {
        stiffness_matrix_1,
        force_vector_1,
        {}
    };

    Eigen::Vector2d expected_displacements_1;
    expected_displacements_1 << 1.0, 2.0;


    // System 2
    Eigen::Matrix2d stiffness_matrix_2;
    stiffness_matrix_2 << 6.0, 2.0,
                          2.0, 4.0;

    Eigen::Vector2d force_vector_2;
    force_vector_2 << 10.0, 8.0;

    const ReducedSystem reduced_system_2
    {
        stiffness_matrix_2,
        force_vector_2,
        {}
    };

    Eigen::Vector2d expected_displacements_2;
    expected_displacements_2 << 1.2, 1.4;


    // System 3
    Eigen::Matrix2d stiffness_matrix_3;
    stiffness_matrix_3 << 5.0, 1.0,
                          1.0, 2.0;

    Eigen::Vector2d force_vector_3;
    force_vector_3 << 7.0, 5.0;

    const ReducedSystem reduced_system_3
    {
        stiffness_matrix_3,
        force_vector_3,
        {}
    };

    Eigen::Vector2d expected_displacements_3;
    expected_displacements_3 << 1.0, 2.0;


    // Solve and compare
    EXPECT_TRUE(
        LinearSystemSolver::solve_reduced_system(reduced_system_1)
            .isApprox(expected_displacements_1));

    EXPECT_TRUE(
        LinearSystemSolver::solve_reduced_system(reduced_system_2)
            .isApprox(expected_displacements_2));

    EXPECT_TRUE(
        LinearSystemSolver::solve_reduced_system(reduced_system_3)
            .isApprox(expected_displacements_3));
}


// handles a fully constrained model
TEST(LinearSystemSolverTests, EmptyReducedSystem)
{
    const Eigen::MatrixXd stiffness_matrix(0, 0);
    const Eigen::VectorXd force_vector(0);

    const DofPartition dof_partition
    {
        {},  // No free DOFs
        {
            {0, 0.0},
            {1, 0.0},
            {2, 0.0},
            {3, 0.0}
        }
    };

    const ReducedSystem reduced_system
    {
        stiffness_matrix,
        force_vector,
        dof_partition
    };

    const Eigen::VectorXd displacements =
        LinearSystemSolver::solve_reduced_system(reduced_system);

    EXPECT_EQ(displacements.size(), 0);
}


// rejects incompatible matrix/vector dimensions
TEST(LinearSystemSolverTests, InvalidDimensions)
{
    // Case 1: Non-square stiffness matrix
    Eigen::Matrix<double, 2, 3> stiffness_matrix_1;
    stiffness_matrix_1 << 1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0;

    Eigen::Vector2d force_vector_1 =
        Eigen::Vector2d::Ones();

    const ReducedSystem reduced_system_1
    {
        stiffness_matrix_1,
        force_vector_1,
        {}
    };

    EXPECT_THROW(
        LinearSystemSolver::solve_reduced_system(reduced_system_1),
        std::invalid_argument
    );


    // Case 2: Force vector has incompatible dimensions
    Eigen::Matrix2d stiffness_matrix_2;
    stiffness_matrix_2 << 4.0, 2.0,
                          2.0, 3.0;

    Eigen::Vector3d force_vector_2 =
        Eigen::Vector3d::Ones();

    const ReducedSystem reduced_system_2
    {
        stiffness_matrix_2,
        force_vector_2,
        {}
    };

    EXPECT_THROW(
        LinearSystemSolver::solve_reduced_system(reduced_system_2),
        std::invalid_argument
    );
}


// rejects invalid numerical input
TEST(LinearSystemSolverTests, NonFiniteInput)
{
    // Case 1: Non-finite stiffness matrix
    Eigen::Matrix2d stiffness_matrix_1;
    stiffness_matrix_1 << 4.0, 2.0,
                          2.0, std::numeric_limits<double>::quiet_NaN();

    Eigen::Vector2d force_vector_1;
    force_vector_1 << 8.0, 8.0;

    const ReducedSystem reduced_system_1
    {
        stiffness_matrix_1,
        force_vector_1,
        {}
    };

    EXPECT_THROW(
        LinearSystemSolver::solve_reduced_system(reduced_system_1),
        std::invalid_argument
    );


    // Case 2: Non-finite force vector
    Eigen::Matrix2d stiffness_matrix_2;
    stiffness_matrix_2 << 4.0, 2.0,
                          2.0, 3.0;

    Eigen::Vector2d force_vector_2;
    force_vector_2 << 8.0, std::numeric_limits<double>::infinity();

    const ReducedSystem reduced_system_2
    {
        stiffness_matrix_2,
        force_vector_2,
        {}
    };

    EXPECT_THROW(
        LinearSystemSolver::solve_reduced_system(reduced_system_2),
        std::invalid_argument
    );
}


// handles factorization failure
// singular: at least one zero eigenvalue
// symmetric but non-positive-definite matrix: at least one negative eigenvalue.
TEST(LinearSystemSolverTests, SingularOrNonPDMatrix)
{
    // Case 1: Singular stiffness matrix
    Eigen::Matrix2d singular_stiffness_matrix;
    singular_stiffness_matrix << 1.0, 2.0,
                                 2.0, 4.0;

    Eigen::Vector2d force_vector_1;
    force_vector_1 << 3.0, 6.0;

    const ReducedSystem reduced_system_1
    {
        singular_stiffness_matrix,
        force_vector_1,
        {}
    };

    EXPECT_THROW(
        LinearSystemSolver::solve_reduced_system(reduced_system_1),
        std::runtime_error
    );


    // Case 2: Symmetric but non-positive-definite stiffness matrix
    Eigen::Matrix2d non_pd_stiffness_matrix;
    non_pd_stiffness_matrix <<  2.0, 1.0,
                                1.0, -1.0;

    Eigen::Vector2d force_vector_2;
    force_vector_2 << 1.0, 1.0;

    const ReducedSystem reduced_system_2
    {
        non_pd_stiffness_matrix,
        force_vector_2,
        {}
    };

    EXPECT_THROW(
        LinearSystemSolver::solve_reduced_system(reduced_system_2),
        std::runtime_error
    );
}