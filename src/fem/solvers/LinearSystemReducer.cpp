#include "../../../include/fem/solvers/LinearSystemReducer.hpp"




ReducedSystem LinearSystemReducer::get_free_displacement_system(
        const ReducedStiffnessMatrix& reduced_stiffness,
        const Eigen::VectorXd& global_force_vector)
{//Kff*uf = (Ff - Kfc uc)
    ReducedSystem reduced_system;
    Eigen::VectorXd reduced_forces_vector; //rhs 
    //compute the forces vector
    
    reduced_forces_vector = 
        get_Ff(global_force_vector, reduced_stiffness.dof_partition) -
        reduced_stiffness.Kfc * get_uc(reduced_stiffness.dof_partition);
    
    return {reduced_stiffness.Kff,
            reduced_forces_vector,
            reduced_stiffness.dof_partition};
}
        

Eigen::VectorXd LinearSystemReducer::get_uc(
    const DofPartition& partition)
{
    Eigen::VectorXd uc(partition.constrained_dofs.size());

    for (std::size_t i = 0; i < partition.constrained_dofs.size(); ++i)
        uc[i] = partition.constrained_dofs[i].prescribed_value;

    return uc;
}


Eigen::VectorXd LinearSystemReducer::get_Ff(
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