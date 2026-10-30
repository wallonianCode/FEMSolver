#include "../../../include/fem/solvers/LinearSystemReducer.hpp"




ReducedSystem LinearSystemReducer::get_free_displacement_system(
        const ReducedStiffnessMatrix& reduced_stiffness,
        const Eigen::VectorXd& global_force_vector)
{
    
}
        



Eigen::VectorXd LinearSystemReducer::get_constrained_displacements(
    const DofPartition& partition)
{
    Eigen::VectorXd uc(partition.constrained_dofs.size());

    for (std::size_t i = 0; i < partition.constrained_dofs.size(); ++i)
        uc[i] = partition.constrained_dofs[i].prescribed_value;

    return uc;
}


Eigen::VectorXd LinearSystemReducer::get_forces_on_free_dofs(
        const Eigen::VectorXd& global_force_vector,
        const DofPartition& partition
)
{
    if (global_force_vector.size() !=
    static_cast<Eigen::Index>(
        partition.free_dofs.size() + partition.constrained_dofs.size()))
    {
        throw std::invalid_argument("Inconsistent DOF partition and force vector dimensions.");
    }

    Eigen::VectorXd forces_on_free_dofs(partition.free_dofs.size());

    for (std::size_t i = 0; i < partition.free_dofs.size(); ++i)
    {
        forces_on_free_dofs[i] =
            global_force_vector[partition.free_dofs[i]];
    }

    return forces_on_free_dofs;
}