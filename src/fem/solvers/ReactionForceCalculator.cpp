#include "../../../include/fem/solvers/ReactionForceCalculator.hpp"


Eigen::VectorXd ReactionForceCalculator::get_reaction_forces
    (const ReducedStiffnessMatrix& reduced_stiffness,
     const Eigen::VectorXd& free_displacements,
     const Eigen::VectorXd& forces_on_constrained_dofs)
{
    if (free_displacements.size() != reduced_stiffness.Kcf.rows())
    {
        throw(std::invalid_argument("Size mismatch between free displacements & \
            global stiffness matrix"));
    }
    //TODO additional tests?
    return reduced_stiffness.Kcf*free_displacements +
           reduced_stiffness.Kcc*
           LinearSystemReducer::get_uc(reduced_stiffness.dof_partition) -
           forces_on_constrained_dofs;
}