#include "../../include/fem/Node.hpp"


Node::Node(std::size_t id, const float x, const float y) : id(id), 
coordinates(x, y) {}