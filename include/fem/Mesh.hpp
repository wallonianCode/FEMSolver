#ifndef MESH_HPP
#define MESH_HPP

#include <vector>
#include <cstddef>
#include <exception>
#include <string>
#include <sstream>

#include "Node.hpp"
#include "Element.hpp"

//ownership, connectivity, validation
class Mesh
{
private:
    std::vector<Node> nodes_;
    std::vector<Tri3> elements_;

public:
    // Add a node and return its internal index.
    std::size_t add_node(double x, double y);

    // Add a triangle using three node indices.
    std::size_t add_tri3(
        std::size_t n0,
        std::size_t n1,
        std::size_t n2
    );

    
    // Read-only access (do not allow arbitrary code to accidentally
    // change a node's coordinates or an element's connectivity while 
    // the stiffness matrices are being computed)
    const Node& node(std::size_t index) const;
    const Tri3& element(std::size_t index) const;

    std::size_t num_nodes() const;
    std::size_t num_elements() const;

    // Mesh's structural & geometric integrity
    bool validate(std::string& error, double tolerance) const;

    // Mesh's suitability for analysis  
    bool is_valid_for_analysis(std::string& error, double tolerance) const;
};

#endif