#ifndef ELEMENT_H
#define ELEMENT_H

#include "Node.hpp"
#include <array>


struct Tri3 {
	std::size_t id;
	std::array<std::size_t, 3> nodes;
};

#endif
