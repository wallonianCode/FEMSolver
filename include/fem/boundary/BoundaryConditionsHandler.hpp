#ifndef BOUNDARY_CONDITIONS_HANDLER_H
#define BOUNDARY_CONDITIONS_HANDLER_H

#include "DofManager.hpp"
#include "ConstrainedDof.hpp"
#include "BoundaryCondition.hpp"

struct BoundaryConditionsHandler 
{
	/**
	 * @brief  gives a global representation of the constrained dofs with associated values
	 * @param[in]  bc  The boundary conditions to be spread in global coordinates
	 * @return  a sorted vector of constrained dofs expressed in global coordinates
	 */
	static std::vector<ConstrainedDof> 
	get_constrained_dofs_vector(const BoundaryCondition& bc);
};

#endif
