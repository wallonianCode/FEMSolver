#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../include/fem/assembly/ElementStiffness.hpp"
#include "../include/fem/Material.hpp"


TEST(ElementStiffnessTests, StiffnessMatrixCorrespondsToAnalyticalSolution)
{
    // 1. Create a mesh with three nodes.
    Mesh mesh;
    const double element_thickness = 1.0;
    const IsotropicLinearElasticMaterial material = {1, 0};
  
    const Eigen::Vector2d coords_node1 = {0, 0};
    const Eigen::Vector2d coords_node2 = {2, 0};
    const Eigen::Vector2d coords_node3 = {0, 1};

    mesh.add_node(coords_node1[0], coords_node1[1]);
    mesh.add_node(coords_node2[0], coords_node2[1]);
    mesh.add_node(coords_node3[0], coords_node3[1]);
    
    // 2. Add one valid Tri3 element.
    mesh.add_tri3(0, 1, 2, element_thickness);
    // 3. Define the material parameters.
    const PlaneStress stress_state(material);
    // 4. Calculate the element stiffness matrix.
    const ElementStiffness::Matrix6d stiffness_matrix = 
        ElementStiffness::get_stiffness_matrix(mesh.element(0),
                                               mesh,
                                               stress_state);
    // 5. Construct the analytically derived expected matrix.
    ElementStiffness::Matrix6d expected_matrix;
    expected_matrix << 
      3.0/4.0,  1.0/4.0, -1.0/4.0, -1.0/4.0, -1.0/2.0,  0.0,
 	    1.0/4.0,  9.0/8.0,  0.0,    -1.0/8.0, -1.0/4.0, -1.0,
 	   -1.0/4.0,  0.0,      1.0/4.0,  0.0,      0.0,      0.0,
 	   -1.0/4.0, -1.0/8.0,  0.0,     1.0/8.0,  1.0/4.0,  0.0,
 	   -1.0/2.0, -1.0/4.0,  0.0,     1.0/4.0,  1.0/2.0,  0.0,
 	    0.0,     -1.0,      0.0,      0.0,      0.0,      1.0;
    // 6. Compare actual and expected matrices.
    EXPECT_TRUE(stiffness_matrix.isApprox(expected_matrix, 1e-12))
		<< "Actual stiffness matrix:\n" << stiffness_matrix 
		<< "\nExpected stiffness matrix:\n" << expected_matrix;
}


TEST(ElementStiffnessTests, StiffnessMatrixIsSymmetric)
{
    // 1. Create a mesh with three nodes.
    Mesh mesh;
    const double element_thickness = 1.0;
    const IsotropicLinearElasticMaterial material = {1, 0};
  
    const Eigen::Vector2d coords_node1 = {0, 0};
    const Eigen::Vector2d coords_node2 = {3, 4};
    const Eigen::Vector2d coords_node3 = {0, 6};

    mesh.add_node(coords_node1[0], coords_node1[1]);
    mesh.add_node(coords_node2[0], coords_node2[1]);
    mesh.add_node(coords_node3[0], coords_node3[1]);
    
    // 2. Add one valid Tri3 element.
    mesh.add_tri3(0, 1, 2, element_thickness);
    // 3. Define the material parameters.
    const PlaneStress stress_state(material);
    // 4. Calculate the element stiffness matrix.
    const ElementStiffness::Matrix6d stiffness_matrix = 
        ElementStiffness::get_stiffness_matrix(mesh.element(0),
                                               mesh,
                                               stress_state);
    EXPECT_TRUE(stiffness_matrix.isApprox(stiffness_matrix.transpose(), 
		1e-12));
}


TEST(ElementStiffnessTests, RigidBodyMovesProduceZeroInternalForces)
{    
    // 1. Create a mesh with three nodes.
    Mesh mesh;
    const double q = 1.0/4.0;
    const double element_thickness = 1.0;
    const IsotropicLinearElasticMaterial material = {1, 0};
  
    const Eigen::Vector2d coords_node1 = {0, 0};
    const Eigen::Vector2d coords_node2 = {2, 0};
    const Eigen::Vector2d coords_node3 = {0, 1};

    mesh.add_node(coords_node1[0], coords_node1[1]);
    mesh.add_node(coords_node2[0], coords_node2[1]);
    mesh.add_node(coords_node3[0], coords_node3[1]);
    
    // 2. Add one valid Tri3 element.
    mesh.add_tri3(0, 1, 2, element_thickness);
    // 3. Define the material parameters.
    const PlaneStress stress_state(material);
    // Create mesh, material and element as in your existing test.

    const ElementStiffness::Matrix6d stiffness_matrix =
        ElementStiffness::get_stiffness_matrix(
            mesh.element(0), mesh, stress_state);

    Eigen::Matrix<double, 6, 1> u_x;
    u_x << 1, 0, 1, 0, 1, 0;

    Eigen::Matrix<double, 6, 1> u_y;
    u_y << 0, 1, 0, 1, 0, 1;

    Eigen::Matrix<double, 6, 1> u_rotation;
    u_rotation << 0, 0, 0, 2, -1, 0;

    const Eigen::Matrix<double, 6, 1> f_x =
        stiffness_matrix * u_x;

    const Eigen::Matrix<double, 6, 1> f_y =
        stiffness_matrix * u_y;

    const Eigen::Matrix<double, 6, 1> f_rotation =
        stiffness_matrix * u_rotation;

    EXPECT_TRUE(f_x.isZero(1e-12));
    EXPECT_TRUE(f_y.isZero(1e-12));
    EXPECT_TRUE(f_rotation.isZero(1e-12));
}


TEST(ElementStiffnessTests, StiffnessMatrixScalesLinearWithYoungsModulus)
{
    // 1. Create a mesh with three nodes.
    Mesh mesh;
    const double element_thickness = 1.0;
    const IsotropicLinearElasticMaterial material1 = {1, 0},
                                         material2 = {2*material1.elasticity_modulus, 0};
  
    const Eigen::Vector2d coords_node1 = {0, 0};
    const Eigen::Vector2d coords_node2 = {2, 0};
    const Eigen::Vector2d coords_node3 = {0, 1};

    mesh.add_node(coords_node1[0], coords_node1[1]);
    mesh.add_node(coords_node2[0], coords_node2[1]);
    mesh.add_node(coords_node3[0], coords_node3[1]);
    
    // 2. Add one valid Tri3 element.
    mesh.add_tri3(0, 1, 2, element_thickness);
    // 3. Define the material parameters.
    const PlaneStress stress_state1(material1), 
                      stress_state2(material2);

    const ElementStiffness::Matrix6d stiffness_matrix1 =
        ElementStiffness::get_stiffness_matrix(
            mesh.element(0), mesh, stress_state1);
    
    
    const ElementStiffness::Matrix6d stiffness_matrix2=
        ElementStiffness::get_stiffness_matrix(
            mesh.element(0), mesh, stress_state2);

    EXPECT_TRUE(stiffness_matrix2.isApprox(2*stiffness_matrix1, 1e-12));
}


TEST(ElementStiffnessTests, StiffnessMatrixScalesLinearWithElementThickness)
{
    // 1. Create a mesh with three nodes.
    Mesh mesh;
    const double element_thickness1 = 1.0,
                 element_thickness2 = 2*element_thickness1;
    const IsotropicLinearElasticMaterial material1 = {1, 0};
  
    const Eigen::Vector2d coords_node1 = {0, 0};
    const Eigen::Vector2d coords_node2 = {2, 0};
    const Eigen::Vector2d coords_node3 = {0, 1};

    const Eigen::Vector2d coords_node4 = {coords_node1[0]+4.0, 0};
    const Eigen::Vector2d coords_node5 = {coords_node2[0]+4.0, 0};
    const Eigen::Vector2d coords_node6 = {coords_node3[0]+4.0, 1};

    mesh.add_node(coords_node1[0], coords_node1[1]);
    mesh.add_node(coords_node2[0], coords_node2[1]);
    mesh.add_node(coords_node3[0], coords_node3[1]);

    mesh.add_node(coords_node4[0], coords_node4[1]);
    mesh.add_node(coords_node5[0], coords_node5[1]);
    mesh.add_node(coords_node6[0], coords_node6[1]);

    // 2. Add one valid Tri3 element.
    mesh.add_tri3(0, 1, 2, element_thickness1);
    mesh.add_tri3(3, 4, 5, element_thickness2);

    // 3. Define the material parameters.
    const PlaneStress stress_state(material1);

    const ElementStiffness::Matrix6d stiffness_matrix1 =
        ElementStiffness::get_stiffness_matrix(
            mesh.element(0), mesh, stress_state);
    
    
    const ElementStiffness::Matrix6d stiffness_matrix2=
        ElementStiffness::get_stiffness_matrix(
            mesh.element(1), mesh, stress_state);

    EXPECT_TRUE(stiffness_matrix2.isApprox(2*stiffness_matrix1, 1e-12));
}


TEST(ElementStiffnessTests, SteelStiffnessMatrix)
{
    // 1. Create a mesh with three nodes.
    Mesh mesh;
    const double element_thickness = 1.0;
    const IsotropicLinearElasticMaterial material = materials::steel;
  
    const Eigen::Vector2d coords_node1 = {0, 0};
    const Eigen::Vector2d coords_node2 = {2, 0};
    const Eigen::Vector2d coords_node3 = {0, 1};

    mesh.add_node(coords_node1[0], coords_node1[1]);
    mesh.add_node(coords_node2[0], coords_node2[1]);
    mesh.add_node(coords_node3[0], coords_node3[1]);
    
    // 2. Add one valid Tri3 element.
    mesh.add_tri3(0, 1, 2, element_thickness);
    // 3. Define the material parameters.
    const PlaneStress stress_state(material);
    // 4. Calculate the element stiffness matrix.
    const ElementStiffness::Matrix6d stiffness_matrix = 
        ElementStiffness::get_stiffness_matrix(mesh.element(0),
                                               mesh,
                                               stress_state);
    // 5. Construct the analytically derived expected matrix.
    const double E = materials::steel.elasticity_modulus;

    ElementStiffness::Matrix6d expected_matrix;
    expected_matrix <<
        60.0/91.0,   5.0/14.0,  -25.0/91.0,  -5.0/26.0,  -5.0/13.0,  -15.0/91.0,
        5.0/14.0, 435.0/364.0, -15.0/91.0,  -5.0/52.0,  -5.0/26.0, -100.0/91.0,
        -25.0/91.0, -15.0/91.0,   25.0/91.0,   0.0,         0.0,        15.0/91.0,
        -5.0/26.0,  -5.0/52.0,    0.0,         5.0/52.0,   5.0/26.0,   0.0,
        -5.0/13.0,  -5.0/26.0,    0.0,         5.0/26.0,   5.0/13.0,   0.0,
        -15.0/91.0, -100.0/91.0,  15.0/91.0,   0.0,         0.0,       100.0/91.0;

    expected_matrix *= E;
    // 6. Compare actual and expected matrices.
    EXPECT_TRUE(stiffness_matrix.isApprox(expected_matrix, 1e-12));
}

