#ifndef NODE_H
#define NODE_H

#include "Libraries.hpp"
#include <cmath>
#include <iostream>

/*
 * Data structure representing a node.
 * No mathematical operation are defined here on it.
 */
struct Node 
{
 	std::size_t id;
	Eigen::Vector2d coordinates;	
	Node(std::size_t id, const double x, const double y);
	//check coordinates
	static bool is_valid(const Node& node);

	friend std::ostream& operator<< (std::ostream& stream, const Node& node);
	
};

std::ostream& operator<< (std::ostream& stream, const Node& node);

#endif
