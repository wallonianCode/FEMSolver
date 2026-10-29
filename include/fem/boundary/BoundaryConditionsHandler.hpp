#ifndef BOUNDARY_CONDITIONS_HANDLER_H
#define BOUNDARY_CONDITIONS_HANDLER_H

#include "ConstrainedDof.hpp"
#include "BoundaryCondition.hpp"

class BoundaryConditionsHandler 
{
	std::vector<ConstrainedDofs> 
	get_constrained_dofs_vector(const BoundaryCondition& bc);
};

#endif
