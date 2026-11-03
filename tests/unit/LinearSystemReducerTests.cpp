#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../include/fem/boundary/BoundaryConditionsHandler.hpp"
#include "../include/fem/solvers/LinearSystemReducer.hpp"


TEST(LinearSystemReducerTests, NoConstrainnedDofsNoForcesApplied)
{
    Mesh mesh;
    BoundaryCondition bc;


    const double thickness = 1.0;
    const PlaneStress material(materials::steel);

    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.0, 2.0);

    mesh.add_tri3(0, 1, 2, thickness);
    mesh.add_tri3(1, 3, 2, thickness);
    const int total_dofs = mesh.num_nodes()*2;

    const std::vector<ConstrainedDof> constrained_dofs = 
    BoundaryConditionsHandler::get_constrained_dofs_vector(bc);

    const Eigen::VectorXd global_forces = Eigen::VectorXd::Zero(total_dofs);

    const Eigen::MatrixXd global_stiffness = 
    GlobalStiffnessAssembler::assemble(mesh, material);


    const DofPartition partition =
        StiffnessMatrixReducer::partition_dofs(
            total_dofs,
            constrained_dofs);

    const ReducedStiffnessMatrix reduced_stiffness =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness,
            partition
        );

    const ReducedSystem reduced_system = 
        LinearSystemReducer::get_free_displacement_system(
            reduced_stiffness,
            global_forces
        );

    
    const ReducedSystem expected_reduced_system =
    {
        global_stiffness,
        Eigen::VectorXd::Zero(total_dofs),
        partition
    };
    

    EXPECT_TRUE(reduced_system.stiffness_matrix.isApprox(
        expected_reduced_system.stiffness_matrix));

    EXPECT_TRUE(reduced_system.force_vector.isApprox(
        expected_reduced_system.force_vector));

    EXPECT_EQ(reduced_system.dof_partition.free_dofs,
        expected_reduced_system.dof_partition.free_dofs);

    EXPECT_TRUE(reduced_system.dof_partition.constrained_dofs.empty());

    ASSERT_EQ(reduced_system.dof_partition.free_dofs.size(), total_dofs);
}


TEST(LinearSystemReducerTests, NoConstrainedDofsForcedApplied)
{
    Mesh mesh;

    const double thickness = 1.0;
    const PlaneStress material(materials::steel);

    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.0, 2.0);

    mesh.add_tri3(0, 1, 2, thickness);
    mesh.add_tri3(1, 3, 2, thickness);

    const Eigen::Index total_dofs =
        static_cast<Eigen::Index>(mesh.num_nodes() * 2);

    const std::vector<ConstrainedDof> constrained_dofs;

    Eigen::VectorXd global_forces(total_dofs);
    global_forces << 10.0, 20.0, 30.0, 40.0,
                     50.0, 60.0, 70.0, 80.0;

    const Eigen::MatrixXd global_stiffness = 
        GlobalStiffnessAssembler::assemble(mesh, material);

    const DofPartition partition =
        StiffnessMatrixReducer::partition_dofs(
            total_dofs, constrained_dofs);

    const ReducedStiffnessMatrix reduced_stiffness =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    const ReducedSystem reduced_system =
        LinearSystemReducer::get_free_displacement_system(
            reduced_stiffness, global_forces);

    const ReducedSystem expected_reduced_system =
    {
        global_stiffness,
        global_forces,
        partition
    };

    EXPECT_TRUE(reduced_system.stiffness_matrix.isApprox(
        expected_reduced_system.stiffness_matrix));

    EXPECT_TRUE(reduced_system.force_vector.isApprox(
        expected_reduced_system.force_vector));

    EXPECT_EQ(reduced_system.dof_partition.free_dofs,
              expected_reduced_system.dof_partition.free_dofs);

    EXPECT_TRUE(reduced_system.dof_partition.constrained_dofs.empty());
}


TEST(LinearSystemReducerTests, MixedFreeConstrainedDofsNoForcesApplied)
{
    Mesh mesh;

    const double thickness = 1.0;
    const PlaneStress material(materials::steel);

    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.0, 2.0);

    mesh.add_tri3(0, 1, 2, thickness);
    mesh.add_tri3(1, 3, 2, thickness);

    const Eigen::Index total_dofs =
        static_cast<Eigen::Index>(mesh.num_nodes() * 2);

    const std::vector<ConstrainedDof> constrained_dofs =
    {
        {0, 0.0},
        {1, 0.0}
    };

    const Eigen::VectorXd global_forces =
        Eigen::VectorXd::Zero(total_dofs);

    const Eigen::MatrixXd global_stiffness = 
        GlobalStiffnessAssembler::assemble(mesh, material);

    const DofPartition partition =
        StiffnessMatrixReducer::partition_dofs(
            total_dofs, constrained_dofs);

    const ReducedStiffnessMatrix reduced_stiffness =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    const ReducedSystem reduced_system =
        LinearSystemReducer::get_free_displacement_system(
            reduced_stiffness, global_forces);

    const ReducedSystem expected_reduced_system =
    {
        reduced_stiffness.Kff,
        Eigen::VectorXd::Zero(6),
        partition
    };

    EXPECT_TRUE(reduced_system.stiffness_matrix.isApprox(
        expected_reduced_system.stiffness_matrix));

    EXPECT_TRUE(reduced_system.force_vector.isApprox(
        expected_reduced_system.force_vector));

    EXPECT_EQ(reduced_system.dof_partition.free_dofs,
              expected_reduced_system.dof_partition.free_dofs);

    EXPECT_EQ(reduced_system.dof_partition.constrained_dofs.size(), 2);
    EXPECT_EQ(reduced_system.dof_partition.free_dofs.size(), 6);
}


TEST(LinearSystemReducerTests, MixedFreeConstrainedDofsForcedApplied)
{
    Mesh mesh;

    const double thickness = 1.0;
    const PlaneStress material(materials::steel);

    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.0, 2.0);

    mesh.add_tri3(0, 1, 2, thickness);
    mesh.add_tri3(1, 3, 2, thickness);

    const Eigen::Index total_dofs =
        static_cast<Eigen::Index>(mesh.num_nodes() * 2);

    const std::vector<ConstrainedDof> constrained_dofs =
    {
        {0, 0.1},
        {1, -0.2}
    };

    Eigen::VectorXd global_forces(total_dofs);
    global_forces << 10.0, 20.0, 30.0, 40.0,
                     50.0, 60.0, 70.0, 80.0;

    const Eigen::MatrixXd global_stiffness = 
        GlobalStiffnessAssembler::assemble(mesh, material);

    const DofPartition partition =
        StiffnessMatrixReducer::partition_dofs(
            total_dofs, constrained_dofs);

    const ReducedStiffnessMatrix reduced_stiffness =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    const ReducedSystem reduced_system =
        LinearSystemReducer::get_free_displacement_system(
            reduced_stiffness, global_forces);

    Eigen::VectorXd expected_uc(2);
    expected_uc << 0.1, -0.2;

    Eigen::VectorXd expected_Ff(6);
    expected_Ff << 30.0, 40.0, 50.0, 60.0, 70.0, 80.0;

    const Eigen::VectorXd expected_reduced_forces =
        expected_Ff - reduced_stiffness.Kfc * expected_uc;

    const ReducedSystem expected_reduced_system =
    {
        reduced_stiffness.Kff,
        expected_reduced_forces,
        partition
    };

    EXPECT_TRUE(reduced_system.stiffness_matrix.isApprox(
        expected_reduced_system.stiffness_matrix));

    EXPECT_TRUE(reduced_system.force_vector.isApprox(
        expected_reduced_system.force_vector));

    EXPECT_EQ(reduced_system.dof_partition.free_dofs,
              expected_reduced_system.dof_partition.free_dofs);

    EXPECT_EQ(reduced_system.dof_partition.constrained_dofs.size(), 2);
    EXPECT_EQ(reduced_system.dof_partition.free_dofs.size(), 6);
}


TEST(LinearSystemReducerTests, AllDofsConstrained)
{
    Mesh mesh;

    const double thickness = 1.0;
    const PlaneStress material(materials::steel);

    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.0, 2.0);

    mesh.add_tri3(0, 1, 2, thickness);
    mesh.add_tri3(1, 3, 2, thickness);

    const Eigen::Index total_dofs =
        static_cast<Eigen::Index>(mesh.num_nodes() * 2);

    const std::vector<ConstrainedDof> constrained_dofs =
    {
        {0, 0.0},
        {1, 0.0},
        {2, 0.1},
        {3, 0.0},
        {4, 0.0},
        {5, -0.1},
        {6, 0.0},
        {7, 0.2}
    };

    Eigen::VectorXd global_forces(total_dofs);
    global_forces << 10.0, 20.0, 30.0, 40.0,
                     50.0, 60.0, 70.0, 80.0;

    const Eigen::MatrixXd global_stiffness = 
        GlobalStiffnessAssembler::assemble(mesh, material);

    const DofPartition partition =
        StiffnessMatrixReducer::partition_dofs(
            total_dofs, constrained_dofs);

    const ReducedStiffnessMatrix reduced_stiffness =
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            global_stiffness, partition);

    const ReducedSystem reduced_system =
        LinearSystemReducer::get_free_displacement_system(
            reduced_stiffness, global_forces);

    const ReducedSystem expected_reduced_system =
    {
        Eigen::MatrixXd(0, 0),
        Eigen::VectorXd::Zero(0),
        partition
    };

    EXPECT_TRUE(reduced_system.stiffness_matrix.isApprox(
        expected_reduced_system.stiffness_matrix));

    EXPECT_TRUE(reduced_system.force_vector.isApprox(
        expected_reduced_system.force_vector));

    EXPECT_TRUE(reduced_system.dof_partition.free_dofs.empty());

    EXPECT_EQ(reduced_system.dof_partition.constrained_dofs.size(),
              total_dofs);
}