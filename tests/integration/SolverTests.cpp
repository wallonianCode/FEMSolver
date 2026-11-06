#include <gtest/gtest.h>
#include <memory>
#include <cmath>


#include "../../include/fem/solvers/Solver.hpp"


namespace
{
    constexpr double E = 200e9;
    constexpr double nu = 0.3;
    constexpr double epsilon = 1e-3;
    constexpr double thickness = 1.0;

    constexpr double tolerance = 1e-9;

    const IsotropicLinearElasticMaterial test_material{E, nu};

    void prescribe_displacement(
        BoundaryCondition& bc,
        std::size_t node_id,
        double ux,
        double uy)
    {
        bc.add_displacement(node_id, DofComponent::X, ux);
        bc.add_displacement(node_id, DofComponent::Y, uy);
    }

    Eigen::Vector3d expected_strain()
    {
        Eigen::Vector3d result;
        result << epsilon, 0.0, 0.0;
        return result;
    }

    Eigen::Vector3d expected_plane_stress()
    {
        Eigen::Vector3d result;

        result << E * epsilon / (1.0 - nu * nu),
                  E * nu * epsilon / (1.0 - nu * nu),
                  0.0;

        return result;
    }

    Eigen::Vector3d expected_plane_strain()
    {
        Eigen::Vector3d result;

        const double factor =
            E / ((1.0 + nu) * (1.0 - 2.0 * nu));

        result << factor * (1.0 - nu) * epsilon,
                  factor * nu * epsilon,
                  0.0;

        return result;
    }

    void expect_all_elements(
        const Result& result,
        std::size_t expected_elements,
        const Eigen::Vector3d& strain,
        const Eigen::Vector3d& stress)
    {
        ASSERT_EQ(result.strains.size(), expected_elements);
        ASSERT_EQ(result.stresses.size(), expected_elements);

        for (std::size_t i = 0; i < expected_elements; ++i)
        {
            EXPECT_TRUE(
                result.strains[i].isApprox(strain, tolerance)
            ) << "Unexpected strain in element " << i;

            EXPECT_TRUE(
                result.stresses[i].isApprox(stress, tolerance)
            ) << "Unexpected stress in element " << i;
        }
    }
}


TEST(SolverTests, EmptyMesh)
{
    Mesh mesh;

    Solver solver(
        mesh,
        std::make_unique<PlaneStress>(test_material)
    );

    EXPECT_THROW(
        solver.solve(),
        std::invalid_argument
    );
}


TEST(SolverTests, SingleElementMeshUnderPlaneStressConditions)
{
    Mesh mesh;

    const auto n0 = mesh.add_node(0.0, 0.0);
    const auto n1 = mesh.add_node(1.0, 0.0);
    const auto n2 = mesh.add_node(0.0, 1.0);

    mesh.add_tri3(n0, n1, n2, thickness);

    BoundaryCondition bc;

    prescribe_displacement(bc, n0, 0.0, 0.0);
    prescribe_displacement(bc, n1, epsilon, 0.0);
    prescribe_displacement(bc, n2, 0.0, 0.0);

    Solver solver(
        mesh,
        std::make_unique<PlaneStress>(test_material)
    );

    solver.set_boundary_conditions(bc);

    const Result result = solver.solve();

    expect_all_elements(
        result,
        1,
        expected_strain(),
        expected_plane_stress()
    );
}


TEST(SolverTests, SingleElementMeshUnderPlaneStrainConditions)
{
    Mesh mesh;

    const auto n0 = mesh.add_node(0.0, 0.0);
    const auto n1 = mesh.add_node(1.0, 0.0);
    const auto n2 = mesh.add_node(0.0, 1.0);

    mesh.add_tri3(n0, n1, n2, thickness);

    BoundaryCondition bc;

    prescribe_displacement(bc, n0, 0.0, 0.0);
    prescribe_displacement(bc, n1, epsilon, 0.0);
    prescribe_displacement(bc, n2, 0.0, 0.0);

    Solver solver(
        mesh,
        std::make_unique<PlaneStrain>(test_material)
    );

    solver.set_boundary_conditions(bc);

    const Result result = solver.solve();

    expect_all_elements(
        result,
        1,
        expected_strain(),
        expected_plane_strain()
    );
}


TEST(SolverTests, SingleElementMeshRigidBodyMotion)
{
    Mesh mesh;

    const auto n0 = mesh.add_node(0.0, 0.0);
    const auto n1 = mesh.add_node(1.0, 0.0);
    const auto n2 = mesh.add_node(0.0, 1.0);

    mesh.add_tri3(n0, n1, n2, thickness);

    constexpr double tx = 0.25;
    constexpr double ty = -0.15;
    constexpr double theta = 0.01;

    BoundaryCondition bc;

    prescribe_displacement(bc, n0, tx, ty);

    prescribe_displacement(
        bc, n1,
        tx - theta * 0.0,
        ty + theta * 1.0
    );

    prescribe_displacement(
        bc, n2,
        tx - theta * 1.0,
        ty + theta * 0.0
    );

    Solver solver(
        mesh,
        std::make_unique<PlaneStress>(test_material)
    );

    solver.set_boundary_conditions(bc);

    const Result result = solver.solve();

    ASSERT_EQ(result.strains.size(), 1);
    ASSERT_EQ(result.stresses.size(), 1);

    EXPECT_TRUE(
        result.strains[0].isZero(tolerance)
    );

    EXPECT_TRUE(
        result.stresses[0].isZero(tolerance)
    );
}

TEST(SolverTests, DoubleElementMeshUnderPlaneStressConditions)
{
    Mesh mesh;

    const auto n0 = mesh.add_node(0.0, 0.0);
    const auto n1 = mesh.add_node(1.0, 0.0);
    const auto n2 = mesh.add_node(1.0, 1.0);
    const auto n3 = mesh.add_node(0.0, 1.0);

    mesh.add_tri3(n0, n1, n2, thickness);
    mesh.add_tri3(n0, n2, n3, thickness);

    BoundaryCondition bc;

    prescribe_displacement(bc, n0, 0.0, 0.0);
    prescribe_displacement(bc, n1, epsilon, 0.0);
    prescribe_displacement(bc, n2, epsilon, 0.0);
    prescribe_displacement(bc, n3, 0.0, 0.0);

    Solver solver(
        mesh,
        std::make_unique<PlaneStress>(test_material)
    );

    solver.set_boundary_conditions(bc);

    const Result result = solver.solve();

    expect_all_elements(
        result,
        2,
        expected_strain(),
        expected_plane_stress()
    );
}


TEST(SolverTests, SingleElementMeshUnloaded)
{

}




TEST(SolverTests, EightElementsMeshUnderPlaneStrainConditions)
{
    Mesh mesh;

    constexpr std::size_t number_of_nodes_per_row = 3;

    std::size_t nodes[3][3];

    // Create a 3 x 3 node grid.
    for (std::size_t y = 0; y < 3; ++y)
    {
        for (std::size_t x = 0; x < 3; ++x)
        {
            nodes[y][x] = mesh.add_node(
                static_cast<double>(x),
                static_cast<double>(y)
            );
        }
    }

    // Divide each of the four squares into two triangles.
    for (std::size_t y = 0; y < 2; ++y)
    {
        for (std::size_t x = 0; x < 2; ++x)
        {
            const auto n00 = nodes[y][x];
            const auto n10 = nodes[y][x + 1];
            const auto n11 = nodes[y + 1][x + 1];
            const auto n01 = nodes[y + 1][x];

            mesh.add_tri3(n00, n10, n11, thickness);
            mesh.add_tri3(n00, n11, n01, thickness);
        }
    }

    ASSERT_EQ(mesh.num_nodes(), 9);
    ASSERT_EQ(mesh.num_elements(), 8);

    BoundaryCondition bc;

    // Prescribe every boundary node.
    // The right edge (x = 2) is clamped.
    // Other boundary nodes follow the affine displacement field.
    for (std::size_t y = 0; y < 3; ++y)
    {
        for (std::size_t x = 0; x < 3; ++x)
        {
            if (x == 1 && y == 1)
                continue; // Interior node remains free.

            const double ux =
                epsilon * (static_cast<double>(x) - 2.0);

            prescribe_displacement(
                bc,
                nodes[y][x],
                ux,
                0.0
            );
        }
    }

    Solver solver(
        mesh,
        std::make_unique<PlaneStrain>(test_material)
    );

    solver.set_boundary_conditions(bc);

    const Result result = solver.solve();

    expect_all_elements(
        result,
        8,
        expected_strain(),
        expected_plane_strain()
    );

    // Check that the interior node has the analytically expected displacement.
    // Node ID 4 is the central node.
    EXPECT_NEAR(
        result.global_displacements(2 * nodes[1][1]),
        -epsilon,
        tolerance
    );

    EXPECT_NEAR(
        result.global_displacements(2 * nodes[1][1] + 1),
        0.0,
        tolerance
    );
}

/*
TEST(SolverTests, ReactionForcesEquilibrium)
{
    //TODO
}
    */

