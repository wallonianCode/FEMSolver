#ifndef NODE_H
#define NODE_H

#include "Libraries.hpp"

/*
 * Data structure representing a node.
 * No mathematical operation are defined here on it.
 */
struct Node 
{
 	std::size_t id;
	Eigen::Vector2d coordinates;	
	Node(std::size_t id, const float x, const float y);
};

#endif
