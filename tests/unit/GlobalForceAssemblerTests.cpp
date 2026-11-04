#include <gtest/gtest.h>
#include <Eigen\Dense>
#include "../../include/fem/assembly/GlobalForceAssembler.hpp"



TEST(GlobalForceAssemblerTests, EmptyMesh)
{
    Mesh mesh;
    LoadConditions load_conditions;

    const Eigen::VectorXd global_force_vector =
        GlobalForceAssembler::assemble(mesh, load_conditions);

    EXPECT_EQ(global_force_vector.size(), 0);
}


TEST(GlobalForceAssemblerTests, EmptyLoadConditions)
{
    Mesh mesh;

    mesh.add_node(0.0, 0.0);
    mesh.add_node(1.0, 0.0);
    mesh.add_node(0.0, 1.0);

    LoadConditions load_conditions;

    const Eigen::VectorXd global_force_vector =
        GlobalForceAssembler::assemble(mesh, load_conditions);

    const Eigen::VectorXd expected_vector =
        Eigen::VectorXd::Zero(6);

    EXPECT_EQ(global_force_vector.size(), 6);
    EXPECT_TRUE(global_force_vector.isApprox(expected_vector));
}


TEST(GlobalForceAssemblerTests, GeneralCase)
{
    Mesh mesh;

    mesh.add_node(0.0, 0.0); // Node 0
    mesh.add_node(2.0, 0.0); // Node 1
    mesh.add_node(0.0, 1.0); // Node 2

    LoadConditions load_conditions;

    // Two loads applied to the X DOF of node 0.
    load_conditions.add_nodal_load(0, DofComponent::X, 10.0);
    load_conditions.add_nodal_load(0, DofComponent::X, 5.0);

    // Load applied to the Y DOF of node 0.
    load_conditions.add_nodal_load(0, DofComponent::Y, -3.0);

    // Load applied to the X DOF of node 1.
    load_conditions.add_nodal_load(1, DofComponent::X, 20.0);

    // Load applied to the Y DOF of node 2.
    load_conditions.add_nodal_load(2, DofComponent::Y, 7.0);

    const Eigen::VectorXd global_force_vector =
        GlobalForceAssembler::assemble(mesh, load_conditions);

    Eigen::VectorXd expected_vector(6);
    expected_vector << 15.0, -3.0, 20.0, 0.0, 0.0, 7.0;

    EXPECT_EQ(global_force_vector.size(), 6);
    EXPECT_TRUE(global_force_vector.isApprox(expected_vector));
}




