#ifndef GLOBAL_DISPLACEMENTS_RECONSTRUCTOR_H
#define GLOBAL_DISPLACEMENTS_RECONSTRUCTOR_H

#include "LinearSystemSolver.hpp"


class GlobalDisplacementsReconstructor
{
public:
    static Eigen::VectorXd 
    get_global_displacements_vector(
        const Eigen::VectorXd& free_displacements,
        const DofPartition& partition
    );
};


#endif