#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../include/fem/assembly/StiffnessMatrixReducer.hpp"


TEST(StiffnessMatrixReducerTests, PartitionNoContraintsDofs)
{
    const Eigen::Index total_dofs = 6;
    const std::vector<ConstrainedDof> constrained_dofs;

    const DofPartition partition =
        StiffnessMatrixReducer::partition_dofs(total_dofs, constrained_dofs);

    EXPECT_TRUE(partition.constrained_dofs.empty());

    ASSERT_EQ(partition.free_dofs.size(), 6);

    for (Eigen::Index i = 0; i < total_dofs; ++i)
        EXPECT_EQ(partition.free_dofs[i], i);
}



TEST(StiffnessMatrixReducerTests, PartitionNoFreeDofs)
{
    const Eigen::Index total_dofs = 4;

    const std::vector<ConstrainedDof> constrained_dofs{
        {0, 0.0},
        {1, 0.0},
        {2, 0.5},
        {3, 1.0}
    };

    const DofPartition partition =
        StiffnessMatrixReducer::partition_dofs(total_dofs, constrained_dofs);

    EXPECT_TRUE(partition.free_dofs.empty());

    ASSERT_EQ(partition.constrained_dofs.size(), 4);

    for (Eigen::Index i = 0; i < total_dofs; ++i)
    {
        EXPECT_EQ(partition.constrained_dofs[i].global_dof, i);
    }
}



TEST(StiffnessMatrixReducerTests, PartitionConstraintsAndFreeDofs)
{
    const Eigen::Index total_dofs = 6;

    const std::vector<ConstrainedDof> constrained_dofs{
        {1, 0.5},
        {4, 1.0}
    };

    const DofPartition partition =
        StiffnessMatrixReducer::partition_dofs(total_dofs, constrained_dofs);

    // Verify free DOFs
    const std::vector<Eigen::Index> expected_free_dofs{
        0, 2, 3, 5
    };

    ASSERT_EQ(partition.free_dofs.size(), expected_free_dofs.size());

    for (std::size_t i = 0; i < expected_free_dofs.size(); ++i)
        EXPECT_EQ(partition.free_dofs[i], expected_free_dofs[i]);

    // Verify constrained DOFs
    ASSERT_EQ(partition.constrained_dofs.size(), 2);

    EXPECT_EQ(partition.constrained_dofs[0].global_dof, 1);
    EXPECT_DOUBLE_EQ(partition.constrained_dofs[0].prescribed_value, 0.5);

    EXPECT_EQ(partition.constrained_dofs[1].global_dof, 4);
    EXPECT_DOUBLE_EQ(partition.constrained_dofs[1].prescribed_value, 1.0);
}



TEST(StiffnessMatrixReducerTests, ReducedStiffnessOfEmptyGlobalStiffnessMatrix)
{
    const Eigen::MatrixXd global_stiffness(0, 0);
    const DofPartition partition;

    const ReducedStiffnessMatrix reduced =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    EXPECT_EQ(reduced.Kff.rows(), 0);
    EXPECT_EQ(reduced.Kff.cols(), 0);

    EXPECT_EQ(reduced.Kfc.rows(), 0);
    EXPECT_EQ(reduced.Kfc.cols(), 0);

    EXPECT_EQ(reduced.Kcf.rows(), 0);
    EXPECT_EQ(reduced.Kcf.cols(), 0);

    EXPECT_EQ(reduced.Kcc.rows(), 0);
    EXPECT_EQ(reduced.Kcc.cols(), 0);

    EXPECT_TRUE(reduced.dof_partition.free_dofs.empty());
    EXPECT_TRUE(reduced.dof_partition.constrained_dofs.empty());
}



TEST(StiffnessMatrixReducerTests, ReducedStiffnessOfNonSymmetricGlobalStiffnessMatrix)
{
    Eigen::MatrixXd global_stiffness(4, 4);

    global_stiffness <<
         1,  2,  3,  4,
         5,  6,  7,  8,
         9, 10, 11, 12,
        13, 14, 15, 16;

    const DofPartition partition{
        {0, 2},
        {
            {1, 0.0},
            {3, 0.0}
        }
    };

    const ReducedStiffnessMatrix reduced =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    Eigen::Matrix2d expected_Kff;
    expected_Kff << 1, 3,
                    9, 11;

    Eigen::Matrix2d expected_Kfc;
    expected_Kfc << 2, 4,
                    10, 12;

    Eigen::Matrix2d expected_Kcf;
    expected_Kcf << 5, 7,
                    13, 15;

    Eigen::Matrix2d expected_Kcc;
    expected_Kcc << 6, 8,
                    14, 16;

    EXPECT_TRUE(reduced.Kff.isApprox(expected_Kff));
    EXPECT_TRUE(reduced.Kfc.isApprox(expected_Kfc));
    EXPECT_TRUE(reduced.Kcf.isApprox(expected_Kcf));
    EXPECT_TRUE(reduced.Kcc.isApprox(expected_Kcc));
}


TEST(StiffnessMatrixReducerTests, ReducedStiffnessMatrixWithOnlyConstrainedDofs)
{
    Eigen::Matrix3d global_stiffness;

    global_stiffness <<
        10,  2,  3,
         2, 20,  4,
         3,  4, 30;

    const DofPartition partition{
        {},
        {
            {0, 0.0},
            {1, 0.0},
            {2, 0.0}
        }
    };

    const ReducedStiffnessMatrix reduced =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    // Kff: 0 x 0
    EXPECT_EQ(reduced.Kff.rows(), 0);
    EXPECT_EQ(reduced.Kff.cols(), 0);

    // Kfc: 0 x 3
    EXPECT_EQ(reduced.Kfc.rows(), 0);
    EXPECT_EQ(reduced.Kfc.cols(), 3);

    // Kcf: 3 x 0
    EXPECT_EQ(reduced.Kcf.rows(), 3);
    EXPECT_EQ(reduced.Kcf.cols(), 0);

    // Kcc: 3 x 3
    EXPECT_EQ(reduced.Kcc.rows(), 3);
    EXPECT_EQ(reduced.Kcc.cols(), 3);

    EXPECT_TRUE(reduced.Kcc.isApprox(global_stiffness));
}


TEST(StiffnessMatrixReducerTests, ReducedStiffnessMatrixWithOnlyFreeDofs)
{
    Eigen::Matrix4d global_stiffness;

    global_stiffness <<
        10,  2,  3,  4,
         2, 20,  5,  6,
         3,  5, 30,  7,
         4,  6,  7, 40;

    const DofPartition partition{
        {0, 1, 2, 3},
        {}
    };

    const ReducedStiffnessMatrix reduced =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    // Kff: 4 x 4
    EXPECT_EQ(reduced.Kff.rows(), 4);
    EXPECT_EQ(reduced.Kff.cols(), 4);
    EXPECT_TRUE(reduced.Kff.isApprox(global_stiffness));

    // Kfc: 4 x 0
    EXPECT_EQ(reduced.Kfc.rows(), 4);
    EXPECT_EQ(reduced.Kfc.cols(), 0);

    // Kcf: 0 x 4
    EXPECT_EQ(reduced.Kcf.rows(), 0);
    EXPECT_EQ(reduced.Kcf.cols(), 4);

    // Kcc: 0 x 0
    EXPECT_EQ(reduced.Kcc.rows(), 0);
    EXPECT_EQ(reduced.Kcc.cols(), 0);
}

TEST(StiffnessMatrixReducerTests, ReducedStiffnessMatrixWithFreeAndConstrainedDofs)
{
    Eigen::Matrix4d global_stiffness;

    global_stiffness <<
        10,  2,  3,  4,
         2, 20,  5,  6,
         3,  5, 30,  7,
         4,  6,  7, 40;

    const DofPartition partition{
        {0, 2},
        {
            {1, 0.0},
            {3, 0.0}
        }
    };

    const ReducedStiffnessMatrix reduced =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    Eigen::Matrix2d expected_Kff;
    expected_Kff << 10, 3,
                     3, 30;

    Eigen::Matrix2d expected_Kfc;
    expected_Kfc << 2, 4,
                    5, 7;

    Eigen::Matrix2d expected_Kcf;
    expected_Kcf << 2, 5,
                    4, 7;

    Eigen::Matrix2d expected_Kcc;
    expected_Kcc << 20, 6,
                     6, 40;

    EXPECT_TRUE(reduced.Kff.isApprox(expected_Kff));
    EXPECT_TRUE(reduced.Kfc.isApprox(expected_Kfc));
    EXPECT_TRUE(reduced.Kcf.isApprox(expected_Kcf));
    EXPECT_TRUE(reduced.Kcc.isApprox(expected_Kcc));

    // Verify that the partition has been retained.
    ASSERT_EQ(reduced.dof_partition.free_dofs.size(), 2);
    EXPECT_EQ(reduced.dof_partition.free_dofs[0], 0);
    EXPECT_EQ(reduced.dof_partition.free_dofs[1], 2);

    ASSERT_EQ(reduced.dof_partition.constrained_dofs.size(), 2);
    EXPECT_EQ(reduced.dof_partition.constrained_dofs[0].global_dof, 1);
    EXPECT_EQ(reduced.dof_partition.constrained_dofs[1].global_dof, 3);
}