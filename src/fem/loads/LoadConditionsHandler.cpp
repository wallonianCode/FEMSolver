#include "../../../include/fem/loads/LoadConditionsHandler.hpp"


std::vector<LoadedDof> 
       LoadConditionsHandler::get_loaded_dofs_vector(const LoadConditions& load_conditions)
{
  std::vector<LoadedDof> loaded_dofs;

  for (const NodalLoad& nodal_load : load_conditions)
  {
    const Eigen::Vector2i global_dofs =  
			DofManager::get_dofs_from_node_id(nodal_load.node_id);
	
    Eigen::Index global_dof;	
    switch (nodal_load.component)
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

    const LoadedDof loaded_dof =
    {
        global_dof,
        nodal_load.value
    };

    loaded_dofs.push_back(loaded_dof);
  }

  std::sort(loaded_dofs.begin(), loaded_dofs.end(), 
  [](const LoadedDof& loaded_dof1, const LoadedDof& loaded_dof2)
  {
    return loaded_dof1.global_dof < loaded_dof2.global_dof;
  });

  
  return loaded_dofs;
}
