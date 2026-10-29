#include <cmath>
#include <stdexcept>

#include "../../../include/fem/boundary/BoundaryCondition.hpp"

// the class only contains a std::vector which already knows how to copy its elements
// therefore, let the compiler generate the copy assignment operator
//BoundaryCondition& BoundaryCondition::operator=(const BoundaryCondition& other) = default;

void BoundaryCondition::add_displacement(
    const std::size_t node_id,
    const DofComponent component,
    const double value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(
            "Prescribed displacement must be finite.");
    }

		//TODO check for duplicates
		//(make use of an std function)
    displacements_.push_back({node_id, component, value});
}


std::vector<PrescribedDisplacement>::const_iterator BoundaryCondition::begin() const noexcept
{
    return displacements_.cbegin();
}


std::vector<PrescribedDisplacement>::const_iterator BoundaryCondition::end() const noexcept
{
    return displacements_.cend();
}
