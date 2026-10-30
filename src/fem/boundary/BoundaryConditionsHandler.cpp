#include <stdexcept>
#include "../../../include/fem/boundary/BoundaryConditionsHandler.hpp"


std::vector<ConstrainedDof> 
BoundaryConditionsHandler::get_constrained_dofs_vector(
const BoundaryCondition& bc) 
{
	std::vector<ConstrainedDof> constrained_dofs;

  	for (const PrescribedDisplacement& displacement : bc)
	{
	  	const Eigen::Vector2i global_dofs =  
			DofManager::get_dofs_from_node_id(displacement.node_id);
	
		Eigen::Index global_dof;	
		switch (displacement.component)
		{
   		  case DofComponent::X:
			global_dof = global_dofs[0];
			break;

		  case DofComponent::Y:
		  	global_dof = global_dofs[1];
				break;

		  default:
				throw std::invalid_argument("Invalid DOF component.");
		}

		const ConstrainedDof constrained_dof {
			global_dof,
			displacement.value
		};

		constrained_dofs.push_back(constrained_dof);
	}

	std::sort(constrained_dofs.begin(), constrained_dofs.end(), 
	[](const ConstrainedDof& dof1, const ConstrainedDof& dof2)
	{
		return dof1.global_dof < dof2.global_dof;
	});

	return constrained_dofs;
}
