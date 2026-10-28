#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Eigen/Dense>
#include <array>
#include <cstddef>

#include "../include/fem/assembly/GlobalStiffnessAssembler.hpp"
#include "../include/fem/assembly/ElementStiffness.hpp"
#include "../include/fem/Material.hpp"
#include "../include/fem/Mesh.hpp"
#include "../include/fem/DofManager.hpp"



TEST(GlobalStiffnessTests, GlobalStiffnessMatrixOneElement)
{
	//global & element stiffness corresponds to each other in this case	
	Mesh mesh;
	ElementStiffness::Matrix6d element_stiffness;
	Eigen::MatrixXd global_stiffness;

	const double thickness = 1.0;
	const IsotropicLinearElasticMaterial material = {1, 0};
	const PlaneStress stress_state(material);
	const DofManager dof_manager;

	mesh.add_node(0.0, 0.0);
	mesh.add_node(2.0, 0.0);
	mesh.add_node(0.0, 1.0);

	mesh.add_tri3(0, 1, 2, thickness);

	element_stiffness = 
	ElementStiffness::get_stiffness_matrix(mesh.element(0),
																				 mesh,
																				 stress_state);

	global_stiffness =
	GlobalStiffnessAssembler::assemble(mesh,
																		 dof_manager,
																		 stress_state);

	ASSERT_EQ(global_stiffness.rows(), 6);
	ASSERT_EQ(global_stiffness.cols(), 6);
	EXPECT_TRUE(global_stiffness.isApprox(element_stiffness, 1e-12));
}


TEST(GlobalStiffnessTests, GlobalStiffnessMatrixTwoElementsTwoCommonNodes)
{
	//elements form together a square
	Mesh mesh;

	const double thickness = 1.0;
	const IsotropicLinearElasticMaterial material = {1, 0};
	const PlaneStress stress_state(material);
	const DofManager dof_manager;

	// Define the four nodes of a unit square.
	mesh.add_node(0.0, 0.0); // n0
	mesh.add_node(1.0, 0.0); // n1
	mesh.add_node(0.0, 1.0); // n2
	mesh.add_node(1.0, 1.0); // n3

	// Create two triangular elements sharing the diagonal n1-n2.
	mesh.add_tri3(0, 1, 2, thickness);
	mesh.add_tri3(1, 3, 2, thickness);

	// Calculate the global stiffness matrix.
	const Eigen::MatrixXd global_stiffness =
			GlobalStiffnessAssembler::assemble(
					mesh,
					dof_manager,
					stress_state);

	// Independently derived reference matrix.
	Eigen::Matrix<double, 8, 8> expected_stiffness;
	expected_stiffness <<
			 3,  1, -2, -1, -1,  0,  0,  0,
			 1,  3,  0, -1, -1, -2,  0,  0,
			-2,  0,  3,  0,  0,  1, -1, -1,
			-1, -1,  0,  3,  1,  0,  0, -2,
			-1, -1,  0,  1,  3,  0, -2,  0,
			 0, -2,  1,  0,  0,  3, -1, -1,
			 0,  0, -1,  0, -2, -1,  3,  1,
			 0,  0, -1, -2,  0, -1,  1,  3;

	expected_stiffness *= 0.25;

	ASSERT_EQ(global_stiffness.rows(), 8);
	ASSERT_EQ(global_stiffness.cols(), 8);

	EXPECT_TRUE(global_stiffness.isApprox(
			expected_stiffness, 1e-12));
}


TEST(GlobalStiffnessTests, GlobalStiffnessMatrixForFourElements)
{
	//elements form together two jointed squares
	Mesh mesh;

	const double thickness = 1.0;
	const IsotropicLinearElasticMaterial material = {1.0, 0.0};
	const PlaneStress stress_state(material);
	const DofManager dof_manager;

	// Bottom row
	mesh.add_node(0.0, 0.0); // n0
	mesh.add_node(1.0, 0.0); // n1
	mesh.add_node(2.0, 0.0); // n2

	// Top row
	mesh.add_node(0.0, 1.0); // n3
	mesh.add_node(1.0, 1.0); // n4
	mesh.add_node(2.0, 1.0); // n5

	// Left square
	mesh.add_tri3(0, 1, 4, thickness);
	mesh.add_tri3(0, 4, 3, thickness);

	// Right square
	mesh.add_tri3(1, 2, 5, thickness);
	mesh.add_tri3(1, 5, 4, thickness);

	const Eigen::MatrixXd global_stiffness =
			GlobalStiffnessAssembler::assemble(
					mesh, dof_manager, stress_state);

	Eigen::Matrix<double, 12, 12> expected_stiffness;

	expected_stiffness <<
			 3,  0, -2,  0,  0,  0, -1,  1,  0, -1,  0,  0,
			 0,  3,  1, -1,  0,  0,  0, -2, -1,  0,  0,  0,
			-2,  1,  6, -1, -2,  0,  0,  0, -2,  1,  0, -1,
			 0, -1, -1,  6,  1, -1,  0,  0,  1, -4, -1,  0,
			 0,  0, -2,  1,  3, -1,  0,  0,  0,  0, -1,  0,
			 0,  0,  0, -1, -1,  3,  0,  0,  0,  0,  1, -2,
			-1,  0,  0,  0,  0,  0,  3, -1, -2,  1,  0,  0,
			 1, -2,  0,  0,  0,  0, -1,  3,  0, -1,  0,  0,
			 0, -1, -2,  1,  0,  0, -2,  0,  6, -1, -2,  1,
			-1,  0,  1, -4,  0,  0,  1, -1, -1,  6,  0, -1,
			 0,  0,  0, -1, -1,  1,  0,  0, -2,  0,  3,  0,
			 0,  0, -1,  0,  0, -2,  0,  0,  1, -1,  0,  3;

	expected_stiffness *= 0.25;

	ASSERT_EQ(global_stiffness.rows(), 12);
	ASSERT_EQ(global_stiffness.cols(), 12);

	EXPECT_TRUE(
			global_stiffness.isApprox(expected_stiffness, 1e-12));
}



TEST(GlobalStiffnessTests, GlobalStiffnessMatrixForDisjointedElements)
{
	//Two elements with no nodes in common
	Mesh mesh;

	const double thickness = 1.0;
	const IsotropicLinearElasticMaterial material = {1.0, 0.0};
	const PlaneStress stress_state(material);
	const DofManager dof_manager;

	// First triangle
	mesh.add_node(0.0, 0.0); // n0
	mesh.add_node(2.0, 0.0); // n1
	mesh.add_node(0.0, 1.0); // n2

	// Second, disjoint triangle
	mesh.add_node(3.0, 0.0); // n3
	mesh.add_node(5.0, 0.0); // n4
	mesh.add_node(3.0, 1.0); // n5

	mesh.add_tri3(0, 1, 2, thickness);
	mesh.add_tri3(3, 4, 5, thickness);

	const Eigen::MatrixXd global_stiffness =
			GlobalStiffnessAssembler::assemble(
					mesh, dof_manager, stress_state);

	// Hard-coded analytical stiffness matrix for either triangle.
	Eigen::Matrix<double, 6, 6> expected_element_stiffness;

	expected_element_stiffness <<
			 3.0/4.0,  1.0/4.0, -1.0/4.0, -1.0/4.0, -1.0/2.0,  0.0,
			 1.0/4.0,  9.0/8.0,  0.0,     -1.0/8.0, -1.0/4.0, -1.0,
			-1.0/4.0,  0.0,      1.0/4.0,  0.0,      0.0,      0.0,
			-1.0/4.0, -1.0/8.0,  0.0,      1.0/8.0,  1.0/4.0,  0.0,
			-1.0/2.0, -1.0/4.0,  0.0,      1.0/4.0,  1.0/2.0,  0.0,
			 0.0,     -1.0,      0.0,      0.0,      0.0,      1.0;

	// Expected global matrix: two independent 6x6 blocks.
	Eigen::Matrix<double, 12, 12> expected_stiffness =
			Eigen::Matrix<double, 12, 12>::Zero();

	expected_stiffness.block<6, 6>(0, 0) =
			expected_element_stiffness;

	expected_stiffness.block<6, 6>(6, 6) =
			expected_element_stiffness;

	ASSERT_EQ(global_stiffness.rows(), 12);
	ASSERT_EQ(global_stiffness.cols(), 12);

	//test the off-diagonal blocks
	//The outer parentheses ensure that the preprocessor treats the entire 
	//expression as a single macro argument.
	EXPECT_TRUE((global_stiffness.block<6, 6>(0, 6).isZero(1e-12)));
	EXPECT_TRUE((global_stiffness.block<6, 6>(6, 0).isZero(1e-12)));

	EXPECT_TRUE(global_stiffness.isApprox(expected_stiffness, 1e-12));
}  
