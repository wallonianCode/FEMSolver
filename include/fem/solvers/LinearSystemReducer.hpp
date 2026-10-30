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


class LinearSystemReducer
{
public:
    static ReducedSystem get_free_displacement_system(
        const ReducedStiffnessMatrix& reduced_stiffness,
        const Eigen::VectorXd& global_force_vector);
private:
    static Eigen::VectorXd get_constrained_displacements(
        const DofPartition& partition);

    static Eigen::VectorXd get_forces_on_free_dofs(
        const Eigen::VectorXd& global_force_vector,
        const DofPartition& partition
    );
};

#endif