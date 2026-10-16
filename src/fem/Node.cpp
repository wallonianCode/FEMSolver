#include "../../include/fem/Node.hpp"


Node::Node(std::size_t id, const double x, const double y) : id(id), 
coordinates(x, y) {}


bool Node::is_valid(const Node& node)
{
    return std::isfinite(node.coordinates[0]) && std::isfinite(node.coordinates[1]);
}


std::ostream& operator<< (std::ostream& stream, const Node& node)
{
    stream << "Node " << node.id
           << " (x = " << node.coordinates[0]
           << ", y = " << node.coordinates[1] << ')';
    return stream;
}