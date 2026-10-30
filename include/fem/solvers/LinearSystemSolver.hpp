#ifndef LINEAR_SYSTEM_SOLVER_H
#define LINEAR_SYSTEM_SOLVER_H

#include "LinearSystemReducer.hpp"

class LinearSystemSolver
{
public:
    Eigen::Vector2d solve_reduced_system(const ReducedSystem& reduced_system);

};


#endif