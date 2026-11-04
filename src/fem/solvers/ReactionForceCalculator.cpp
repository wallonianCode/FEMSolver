#include "../../../include/fem/solvers/ReactionForceCalculator.hpp"


Eigen::VectorXd ReactionForceCalculator::get_reaction_forces
    (const ReducedStiffnessMatrix& reduced_stiffness,
     const Eigen::VectorXd& free_displacements,
     const Eigen::VectorXd& global_force_vector)
{
         
    const Eigen::Index total_dofs =
        static_cast<Eigen::Index>(
            reduced_stiffness.dof_partition.free_dofs.size() + 
            reduced_stiffness.dof_partition.constrained_dofs.size()
        );

    if (global_force_vector.size() != total_dofs)
    {
        throw std::invalid_argument(
            "Global force vector size does not match the DOF partition"
        );
    }

    const Eigen::VectorXd uc =
        LinearSystemReducer::get_uc(reduced_stiffness.dof_partition);

    const Eigen::VectorXd fc = 
        ReactionForceCalculator::get_Fc(global_force_vector, reduced_stiffness.dof_partition);
        

    if (!reduced_stiffness.Kff.allFinite() ||
        !reduced_stiffness.Kfc.allFinite() ||
        !reduced_stiffness.Kcf.allFinite() ||
        !reduced_stiffness.Kcc.allFinite())
    {
        throw std::invalid_argument(
            "Reduced stiffness matrices must contain only finite values.");
    }


    if (!free_displacements.allFinite() ||
        !global_force_vector.allFinite() ||
        !uc.allFinite())
    {
        throw std::invalid_argument(
            "Displacement and force vectors must contain only finite values.");
    }
   

    if (free_displacements.size() != reduced_stiffness.Kcf.cols())
    {
        throw(std::invalid_argument("Size mismatch between free displacements vector & \
            Kcf matrix"));
    }

    if (reduced_stiffness.Kcc.cols() != reduced_stiffness.Kcc.rows())
    {
        throw(std::invalid_argument("Kcc matrix must be square"));
    }

    if (reduced_stiffness.Kcc.cols() != uc.size())
    {
        throw(std::invalid_argument("Kcc size must be compatible with uc's"));
    }

    if (reduced_stiffness.Kcf.rows() != reduced_stiffness.Kcc.rows())
    {
        throw(std::invalid_argument("Kcf rows should match Kcc rows"));
    }

    if (fc.size() != reduced_stiffness.Kcc.rows())
    {
        throw std::invalid_argument(
            "Forces on constrained DOFs vector has incompatible size.");
    }

   

    //TODO additional tests?
    return reduced_stiffness.Kcf*free_displacements +
           reduced_stiffness.Kcc* uc -
           fc;
}


Eigen::VectorXd ReactionForceCalculator::get_Fc(
    const Eigen::VectorXd& global_force_vector,
    const DofPartition& partition)
{
    Eigen::VectorXd Fc(partition.constrained_dofs.size());

    for (std::size_t i = 0;
         i < partition.constrained_dofs.size(); ++i)
    {
        const Eigen::Index global_dof =
            partition.constrained_dofs[i].global_dof;

        Fc(static_cast<Eigen::Index>(i)) =
            global_force_vector(global_dof);
    }

    return Fc;
}