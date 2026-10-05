#include "../include/fem/Node.hpp"
#include "../include/fem/Element.hpp"
#include "../include/fem/DofManager.hpp"
#include "../include/fem/Material.hpp"

#include <iostream>


int main()
{
	Node center(0.0f, 0.0f);
	Eigen::Vector2i dofsCenterNode;

	dofsCenterNode = DofManager::get_dofs_from_node(center);

	std::cout << "Dofs for center node: " << dofsCenterNode << std::endl;

	std::cout << "Young modulus for steel: " << Steel::E << std::endl;
	return 0;
}
