#include "../../include/fem/Mesh.hpp"

  // Add a node and return its internal index.
std::size_t Mesh::add_node(double x, double y) 
{
    const std::size_t node_id = nodes_.size();
    nodes_.push_back(Node(node_id, x, y));
    return node_id;   
}

// Add a triangle using three node indices.
std::size_t Mesh::add_tri3(
    std::size_t n0,
    std::size_t n1,
    std::size_t n2
) 
{
    //TODO
}

// Read-only access (do not allow arbitrary code to accidentally
// change a node's coordinates or an element's connectivity while 
// the stiffness matrices are being computed)
const Node& Mesh::node(std::size_t index) const 
{
    //TODO
}
const Tri3& Mesh::element(std::size_t index) const 
{
    //TODO
}

std::size_t Mesh::num_nodes() const 
{
    //TODO
}

std::size_t Mesh::num_elements() const 
{
    //TODO
}

// Validate connectivity and geometry.
void Mesh::validate() const 
{
    //TODO
}
