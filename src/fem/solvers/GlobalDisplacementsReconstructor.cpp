#include "../../../include/fem/solvers/GlobalDisplacementsReconstructor.hpp"


Eigen::VectorXd 
    GlobalDisplacementsReconstructor::get_global_displacements_vector(
        const Eigen::VectorXd& free_displacements,
        const DofPartition& partition
    )
{
    unsigned count;
    if (free_displacements.size() != 
        static_cast<Eigen::Index>(partition.free_dofs.size()))
    {
        throw(std::invalid_argument(
            "GlobalDisplacementReconstructor::get_global_displacement_vector \
             Size of free displacement vector does not match given partition."));
    }

    Eigen::VectorXd u = 
    Eigen::VectorXd::Zero(partition.free_dofs.size() + 
                          partition.constrained_dofs.size());
    const Eigen::VectorXd uc = LinearSystemReducer::get_uc(partition);

    count = 0;
    for (const Eigen::Index& free_dof : partition.free_dofs)
    {
        u(free_dof) = free_displacements[count];
        ++count;
    }

    for (const ConstrainedDof& constrained_dof : partition.constrained_dofs)
    {
        u(constrained_dof.global_dof) = constrained_dof.prescribed_value;
    }

    return u;
}