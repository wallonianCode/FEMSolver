#ifndef ELEMENT_H
#define ELEMENT_H

#include "Node.hpp"
#include <array>


struct Tri3 {
	std::size_t id;
	std::array<Node, 3> nodes;
};

#endif
