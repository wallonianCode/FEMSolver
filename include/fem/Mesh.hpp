#ifndef MESH_HPP
#define MESH_HPP

#include <vector>
#include <cstddef>
#include <exception>
#include <string>
#include <sstream>
#include <algorithm>

#include "Node.hpp"
#include "Element.hpp"

//ownership, connectivity, validation
class Mesh
{
private:
    std::vector<Node> nodes_;
    std::vector<Tri3> elements_;

public:
    using const_iterator = std::vector<Tri3>::const_iterator;
    // Add a node and return its internal index.
    std::size_t add_node(const double x, const double y);


    // Add a triangle using three node indices.
    std::size_t add_tri3(
        std::size_t n0,
        std::size_t n1,
        std::size_t n2,
        const double thickness
    );

    
    // Read-only access (do not allow arbitrary code to accidentally
    // change a node's coordinates or an element's connectivity while 
    // the stiffness matrices are being computed)
    const Node& node(const std::size_t index) const;
    const Tri3& element(const std::size_t index) const;

    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    std::size_t num_nodes() const;
    std::size_t num_elements() const;

    // Mesh's structural & geometric integrity
    bool validate(std::string& error, const double tolerance) const;

    // Mesh's suitability for analysis  
    bool is_valid_for_analysis(std::string& error, const double tolerance) const;
};

#endif