#include "../../../include/fem/solvers/ReactionForceCalculator.hpp"


Eigen::VectorXd ReactionForceCalculator::get_reaction_forces
    (const ReducedStiffnessMatrix& reduced_stiffness,
     const Eigen::VectorXd& free_displacements,
     const Eigen::VectorXd& forces_on_constrained_dofs)
{
         
    const Eigen::VectorXd uc =
        LinearSystemReducer::get_uc(reduced_stiffness.dof_partition);

    if (!reduced_stiffness.Kff.allFinite() ||
        !reduced_stiffness.Kfc.allFinite() ||
        !reduced_stiffness.Kcf.allFinite() ||
        !reduced_stiffness.Kcc.allFinite())
    {
        throw std::invalid_argument(
            "Reduced stiffness matrices must contain only finite values.");
    }


    if (!free_displacements.allFinite() ||
        !forces_on_constrained_dofs.allFinite() ||
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

    if (forces_on_constrained_dofs.size() != reduced_stiffness.Kcc.rows())
    {
        throw std::invalid_argument(
            "Forces on constrained DOFs vector has incompatible size.");
    }

   

    //TODO additional tests?
    return reduced_stiffness.Kcf*free_displacements +
           reduced_stiffness.Kcc* uc -
           forces_on_constrained_dofs;
}