#ifndef STIFFNESS_MATRIX_REDUCER_H
#define STIFFNESS_MATRIX_REDUCER_H

#include <numeric>

#include "GlobalStiffnessAssembler.hpp"
#include "../boundary/ConstrainedDof.hpp"
#include "../boundary/BoundaryCondition.hpp"

//partitions the DOFs and extract the stiffness matrices.

struct DofPartition
{
	std::vector<Eigen::Index> free_dofs;
	std::vector<ConstrainedDof> constrained_dofs;
};


struct ReducedStiffnessMatrix
{
    Eigen::MatrixXd Kff;
    Eigen::MatrixXd Kfc;
    Eigen::MatrixXd Kcf;
    Eigen::MatrixXd Kcc;

    DofPartition dof_partition;
};


class StiffnessMatrixReducer
{
public:
	static DofPartition 
	partition_dofs(
		const Eigen::Index total_dofs,
		const std::vector<ConstrainedDof>& constrained_dofs
	);

    static ReducedStiffnessMatrix 
        get_reduced_stiffness_matrix(const Eigen::MatrixXd& global_stiffness,
                                     const DofPartition& partition);
    
};


#endif