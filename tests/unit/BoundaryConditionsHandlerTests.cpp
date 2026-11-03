
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../include/fem/boundary/BoundaryConditionsHandler.hpp"
#include "../include/fem/Mesh.hpp"


TEST(BoundaryConditionsHandlerTest, EmptyBoundaryConditions)
{
    //get back an empty vector corresponding to empty boundary conditions
    BoundaryCondition bc;
    const std::vector<ConstrainedDof> constrained_dofs = 
        BoundaryConditionsHandler::get_constrained_dofs_vector(bc);
    EXPECT_TRUE(constrained_dofs.empty());
}


TEST(BoundaryConditionsHandlerTests, SingleTriangleElementClamped)
{
    //get back a vector of constrained dofs corresponding to an encastred triangle element
    //(four constrained dof to zero, sorted vector)

    Mesh mesh;
    BoundaryCondition bc;
   
    const double thickness = 1.0;
    const std::vector<ConstrainedDof> expected_constrained_dofs = 
    {ConstrainedDof{0, 0.0}, ConstrainedDof{1, 0.0}, 
     ConstrainedDof{4, 0.0}, ConstrainedDof{5, 0.0}};

    mesh.add_node(0.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.5, 0);

    mesh.add_tri3(0, 1, 2, thickness);

    //deliberately add the conditions in unsorted order
    bc.add_displacement(0, DofComponent::X, 0.0);
    bc.add_displacement(2, DofComponent::X, 0.0);
    bc.add_displacement(0, DofComponent::Y, 0.0);
    bc.add_displacement(2, DofComponent::Y, 0.0);

    const std::vector<ConstrainedDof> constrained_dofs = 
    BoundaryConditionsHandler::get_constrained_dofs_vector(bc);

    EXPECT_EQ(constrained_dofs, expected_constrained_dofs);
}



TEST(BoundaryConditionsHandlerTest, RectangleOfTwoTrianglesSharingCommonEdgeClamped)
{
    Mesh mesh;
    BoundaryCondition bc;

    mesh.add_node(0.0, 0.0);  // Node 0
    mesh.add_node(0.0, 2.0);  // Node 1
    mesh.add_node(1.5, 0.0);  // Node 2
    mesh.add_node(1.5, 2.0);  // Node 3

    mesh.add_tri3(0, 1, 2, 1.0);
    mesh.add_tri3(2, 1, 3, 1.0);

    const std::vector<ConstrainedDof> expected_constrained_dofs{
        {0, 0.0},
        {1, 0.0},
        {4, 0.0},
        {5, 0.0}
    };

    // Deliberately unsorted insertion order.
    bc.add_displacement(2, DofComponent::Y, 0.0);
    bc.add_displacement(0, DofComponent::X, 0.0);
    bc.add_displacement(2, DofComponent::X, 0.0);
    bc.add_displacement(0, DofComponent::Y, 0.0);

    const std::vector<ConstrainedDof> constrained_dofs =
        BoundaryConditionsHandler::get_constrained_dofs_vector(bc);

    EXPECT_EQ(constrained_dofs, expected_constrained_dofs);
}


TEST(BoundaryConditionsHandlerTest, TwoTriangleWithCommonNodesPrescribedDisplacement)
{
    Mesh mesh;
    BoundaryCondition bc;

    mesh.add_node(0.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.5, 0.0);
    mesh.add_node(1.5, 2.0);

    mesh.add_tri3(0, 1, 2, 1.0);
    mesh.add_tri3(2, 1, 3, 1.0);

    const std::vector<ConstrainedDof> expected_constrained_dofs{
        {0, 0.0},
        {1, 0.25},
        {2, 0.5},
        {5, 0.75},
        {6, 1.0},
        {7, 0.0}
    };

    // Deliberately unsorted insertion order.
    bc.add_displacement(3, DofComponent::Y, 0.0);
    bc.add_displacement(2, DofComponent::Y, 0.75);
    bc.add_displacement(0, DofComponent::Y, 0.25);
    bc.add_displacement(3, DofComponent::X, 1.0);
    bc.add_displacement(1, DofComponent::X, 0.5);
    bc.add_displacement(0, DofComponent::X, 0.0);

    const std::vector<ConstrainedDof> constrained_dofs =
        BoundaryConditionsHandler::get_constrained_dofs_vector(bc);

    EXPECT_EQ(constrained_dofs, expected_constrained_dofs);
}



TEST(BoundaryConditionsHandlerTest, MultipleBoundaryConditionsOnSharedNode)
{
    Mesh mesh;
    BoundaryCondition bc;

    mesh.add_node(0.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.5, 0.0);
    mesh.add_node(1.5, 2.0);

    mesh.add_tri3(0, 1, 2, 1.0);
    mesh.add_tri3(2, 1, 3, 1.0);

    const std::vector<ConstrainedDof> expected_constrained_dofs{
        {2, 0.5},
        {3, 0.25}
    };

    bc.add_displacement(1, DofComponent::Y, 0.25);
    bc.add_displacement(1, DofComponent::X, 0.5);

    const std::vector<ConstrainedDof> constrained_dofs =
        BoundaryConditionsHandler::get_constrained_dofs_vector(bc);

    EXPECT_EQ(constrained_dofs, expected_constrained_dofs);
}