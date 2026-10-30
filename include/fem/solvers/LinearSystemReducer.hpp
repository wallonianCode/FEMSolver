#ifndef LINEAR_SYSTEM_REDUCER_H
#define LINEAR_SYSTEM_REDUCER_H

#include "../assembly/StiffnessMatrixReducer.hpp"

//constructs the reduced system Kff uf = Ff - Kfc uc

struct AssembledSystem
{
    Eigen::MatrixXd stiffness_matrix;
    Eigen::VectorXd force_vector;
};

struct ReducedSystem
{
    Eigen::MatrixXd stiffness_matrix;
    Eigen::VectorXd force_vector;
    DofPartition dof_partition;
};


#endif