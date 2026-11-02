#ifndef REACTION_FORCE_CALCULATOR_H
#define REACTION_FORCE_CALCULATOR_H


#include "GlobalDisplacementsReconstructor.hpp"


class ReactionForceCalculator
{
public:
    static Eigen::VectorXd get_reaction_forces
    (const ReducedStiffnessMatrix& reduced_stiffness,
     const Eigen::VectorXd& free_displacements,
     const Eigen::VectorXd& forces_on_constrained_dofs);
};

#endif