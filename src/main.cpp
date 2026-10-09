#include <iostream>

#include "../include/fem/Node.hpp"
#include "../include/fem/Element.hpp"
#include "../include/fem/DofManager.hpp"
#include "../include/fem/Material.hpp"


int main()
{
	Node center(0, 0.0f, 0.0f);
	Eigen::Vector2i dofsCenterNode;

	dofsCenterNode = DofManager::get_dofs_from_node(center);

	std::cout << "Dofs for center node: " << dofsCenterNode << std::endl;


	//--------------------- MATERIAL –----------------------
	PlaneStress model1{materials::steel};
	
	std::cout << "Constitutive matrix for steel: \n" << model1.get_constitutive_matrix() << 
	std::endl;

	//--------------------- DOF Manager --------------------

	Node node1(1, 20.0f, 12.0f);
	std::cout << "Dofs for second node: \n" << 
		DofManager::get_dofs_from_node(node1) << std::endl;

	return 0;
}
