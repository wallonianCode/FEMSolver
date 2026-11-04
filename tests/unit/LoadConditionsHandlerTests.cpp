#include <gtest/gtest.h>
#include "../../include/fem/loads/LoadConditionsHandler.hpp"
#include "../../include/fem/Mesh.hpp"


TEST(LoadConditionsHandlerTest, EmptyLoadConditionss)
{
    //get back an empty vector corresponding to empty load conditions
    LoadConditions lc;
    const std::vector<LoadedDof> loaded_dofs = 
        LoadConditionsHandler::get_loaded_dofs_vector(lc);
    EXPECT_TRUE(loaded_dofs.empty());
}


TEST(LoadConditionsHandlerTests, SingleTriangleElementClamped)
{
    //get back a vector of constrained dofs corresponding to an encastred triangle element
    //(four constrained dof to zero, sorted vector) 
    Mesh mesh;
    LoadConditions lc;
   
    const double thickness = 1.0;
    const std::vector<LoadedDof> expected_loaded_dofs = 
    {LoadedDof{0, 0.0}, LoadedDof{1, 0.0}, 
     LoadedDof{4, 0.0}, LoadedDof{5, 0.0}};

    mesh.add_node(0.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.5, 0);

    mesh.add_tri3(0, 1, 2, thickness);

    //deliberately add the conditions in unsorted order
    lc.add_nodal_load(0, DofComponent::X, 0.0);
    lc.add_nodal_load(2, DofComponent::X, 0.0);
    lc.add_nodal_load(0, DofComponent::Y, 0.0);
    lc.add_nodal_load(2, DofComponent::Y, 0.0);

    const std::vector<LoadedDof> loaded_dofs = 
    LoadConditionsHandler::get_loaded_dofs_vector(lc);

    EXPECT_EQ(loaded_dofs, expected_loaded_dofs);
}



TEST(LoadConditionsHandlerTest, RectangleOfTwoTrianglesSharingCommonEdgeClamped)
{
    Mesh mesh;
    LoadConditions lc;

    mesh.add_node(0.0, 0.0);  // Node 0
    mesh.add_node(0.0, 2.0);  // Node 1
    mesh.add_node(1.5, 0.0);  // Node 2
    mesh.add_node(1.5, 2.0);  // Node 3

    mesh.add_tri3(0, 1, 2, 1.0);
    mesh.add_tri3(2, 1, 3, 1.0);

    const std::vector<LoadedDof> expected_loaded_dofs{
        {0, 0.0},
        {1, 0.0},
        {4, 0.0},
        {5, 0.0}
    };

    // Deliberately unsorted insertion order.
    lc.add_nodal_load(2, DofComponent::Y, 0.0);
    lc.add_nodal_load(0, DofComponent::X, 0.0);
    lc.add_nodal_load(2, DofComponent::X, 0.0);
    lc.add_nodal_load(0, DofComponent::Y, 0.0);

    const std::vector<LoadedDof> loaded_dofs =
        LoadConditionsHandler::get_loaded_dofs_vector(lc);

    EXPECT_EQ(loaded_dofs, expected_loaded_dofs);
}


TEST(LoadConditionsHandlerTest, TwoTriangleWithCommonNodesPrescribedDisplacement)
{
    Mesh mesh;
    LoadConditions lc;

    mesh.add_node(0.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.5, 0.0);
    mesh.add_node(1.5, 2.0);

    mesh.add_tri3(0, 1, 2, 1.0);
    mesh.add_tri3(2, 1, 3, 1.0);

    const std::vector<LoadedDof> expected_loaded_dofs{
        {0, 0.0},
        {1, 0.25},
        {2, 0.5},
        {5, 0.75},
        {6, 1.0},
        {7, 0.0}
    };

    // Deliberately unsorted insertion order.
    lc.add_nodal_load(3, DofComponent::Y, 0.0);
    lc.add_nodal_load(2, DofComponent::Y, 0.75);
    lc.add_nodal_load(0, DofComponent::Y, 0.25);
    lc.add_nodal_load(3, DofComponent::X, 1.0);
    lc.add_nodal_load(1, DofComponent::X, 0.5);
    lc.add_nodal_load(0, DofComponent::X, 0.0);

    const std::vector<LoadedDof> loaded_dofs =
        LoadConditionsHandler::get_loaded_dofs_vector(lc);

    EXPECT_EQ(loaded_dofs, expected_loaded_dofs);
}



TEST(LoadConditionsHandlerTest, MultipleLoadConditionssOnSharedNode)
{
    Mesh mesh;
    LoadConditions lc;

    mesh.add_node(0.0, 0.0);
    mesh.add_node(0.0, 2.0);
    mesh.add_node(1.5, 0.0);
    mesh.add_node(1.5, 2.0);

    mesh.add_tri3(0, 1, 2, 1.0);
    mesh.add_tri3(2, 1, 3, 1.0);

    const std::vector<LoadedDof> expected_loaded_dofs{
        {2, 0.5},
        {3, 0.25}
    };

    lc.add_nodal_load(1, DofComponent::Y, 0.25);
    lc.add_nodal_load(1, DofComponent::X, 0.5);

    const std::vector<LoadedDof> loaded_dofs =
        LoadConditionsHandler::get_loaded_dofs_vector(lc);

    EXPECT_EQ(loaded_dofs, expected_loaded_dofs);
}