#ifndef NODE_H
#define NODE_H

#include "Libraries.hpp"

/*
 * Data structure representing a node.
 * No mathematical operation are defined here on it.
 */
struct Node 
{
private:
	static std::size_t index;

public:
	std::size_t id;
	Eigen::Vector2f coordinates;	
	Node(const float& x, const float& y);
};

#endif
