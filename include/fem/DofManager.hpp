#ifndef DOFMANAGER_H
#include "Node.hpp"


struct DofManager 
{
	static Eigen::Vector2i get_dofs_from_node(const Node& node);
};

#endif
