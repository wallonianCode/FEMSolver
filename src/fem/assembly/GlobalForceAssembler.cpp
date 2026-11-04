#include "../../../include/fem/assembly/GlobalForceAssembler.hpp"

Eigen::VectorXd GlobalForceAssembler::assemble(
    const Mesh& mesh,
    const LoadConditions& load_conditions)
{
    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(mesh.num_nodes()*2);

    for (const NodalLoad& load : load_conditions)
    {
        const Eigen::Vector2i global_dofs =
            DofManager::get_dofs_from_node_id(load.node_id);
        Eigen::Index global_dof;

        switch(load.component)
        {
          case DofComponent::X:
          {
            global_dof = global_dofs[0];
            break;
          }
          case DofComponent::Y:
          {
            global_dof = global_dofs[1];
            break;
          }
          default:
            throw std::invalid_argument("Dof component not recognized");
        }
               
        global_force_vector(global_dof) += load.value;
    }

    return global_force_vector;
}
