#ifndef DOFMANAGER_H
#define DOFMANAGER_H

#include "Node.hpp"


struct DofManager 
{
	static Eigen::Vector2i get_dofs_from_node(const Node& node);
	static Eigen::Vector2i get_dofs_from_node_id(const size_t& node_id);
};

#endif
