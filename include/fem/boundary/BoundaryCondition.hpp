#pragma once

#include <cstddef>
#include <vector>

#include "../DofComponent.hpp"



struct PrescribedDisplacement
{
    std::size_t node_id;
    DofComponent component;
    double value;
};



class BoundaryCondition
{
public:
    using const_iterator =
          std::vector<PrescribedDisplacement>::const_iterator;

    void add_displacement(
        const std::size_t node_id,
        const DofComponent component,
        const double value);

    // const: iteration works on both const and non-const objects
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

private:
    std::vector<PrescribedDisplacement> displacements_;
};