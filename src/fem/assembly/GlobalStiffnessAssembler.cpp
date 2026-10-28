#include "../../../include/fem/assembly/GlobalStiffnessAssembler.hpp"
#include <array>


Eigen::MatrixXd GlobalStiffnessAssembler::assemble(
        const Mesh& mesh,
        const DofManager& dof_manager,
        const LinearElastic2D& material 
    )
{
    const Eigen::Index n_dofs =
        static_cast<Eigen::Index>(2*mesh.num_nodes());

    Eigen::MatrixXd global_stiffness = 
        Eigen::MatrixXd::Zero(n_dofs, n_dofs);

    ElementStiffness::Matrix6d element_stiffness;

    for (Mesh::const_iterator ci_elem = mesh.begin(); ci_elem != mesh.end(); ci_elem++)
    {
        const size_t node_id1 = ci_elem->get_node_id(Tri3NodePos::first);
        const size_t node_id2 = ci_elem->get_node_id(Tri3NodePos::second);
        const size_t node_id3 = ci_elem->get_node_id(Tri3NodePos::third);

        element_stiffness = ElementStiffness::get_stiffness_matrix(*ci_elem, mesh, material);

        const Eigen::Vector2i dof_node1 = dof_manager.get_dofs_from_node_id(node_id1);
        const Eigen::Vector2i dof_node2 = dof_manager.get_dofs_from_node_id(node_id2);
        const Eigen::Vector2i dof_node3 = dof_manager.get_dofs_from_node_id(node_id3);

        const std::array<Eigen::Index, 6> global_dofs = {
            dof_node1[0], dof_node1[1],
            dof_node2[0], dof_node2[1],
            dof_node3[0], dof_node3[1],
        };

        for (Eigen::Index i = 0; i < 6; ++i)
        {
            for (Eigen::Index j = 0; j < 6; ++j)
            {
                global_stiffness(global_dofs[i], global_dofs[j]) +=
                    element_stiffness(i, j);
            }
        }
    }

    return global_stiffness;
}
