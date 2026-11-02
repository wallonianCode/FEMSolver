#ifndef REACTION_FORCES_CALCULATOR_H
#define REACTION_FORCES_CALCULATOR_H


#include "GlobalDisplacementsReconstructor.hpp"


class ReactionForcesCalculator
{
public:
    static Eigen::VectorXd compute_reaction_forces
    (const Eigen::MatrixXd& global_stiffness,
     const Eigen::VectorXd& free_displacements,
     const Eigen::VectorXd& forces_on_constrained_dofs,
     const DofPartition& partition);
};

#endif