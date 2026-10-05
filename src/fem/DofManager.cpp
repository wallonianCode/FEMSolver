#include "../../include/fem/DofManager.hpp"


Eigen::Vector2i DofManager::get_dofs_from_node(const Node& node)
{
	return {node.id*2, node.id*2+1};
}
