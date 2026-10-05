#include "../../include/fem/Node.hpp"


std::size_t Node::index = 0;

Node::Node(const float& x, const float& y) : id(index++), 
coordinates(x, y) {}
