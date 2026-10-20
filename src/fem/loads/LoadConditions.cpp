#include <cmath>
#include <stdexcept>


#include "../../include/fem/LoadConditions.hpp"


void LoadConditions::add_nodal_load(
    const std::size_t node_id,
    const DofComponent component,
    const double value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(
            "Nodal load value must be finite.");
    }

    loads_.push_back({node_id, component, value});
}


//TODO

