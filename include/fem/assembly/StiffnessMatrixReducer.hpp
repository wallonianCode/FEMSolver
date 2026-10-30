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


class StiffnessMatrixReducer
{
public:
	static DofPartition 
	partition_dofs(
		const Eigen::Index total_dofs,
		const std::vector<ConstrainedDof>& constrained_dofs
	);


    static Eigen::MatrixXd get_Kff(const Eigen::MatrixXd& global_stiffness,
                                   const DofPartition& partition);

    static Eigen::MatrixXd get_Kfc(const Eigen::MatrixXd& global_stiffness,
                                   const DofPartition& partition);
    
    static Eigen::MatrixXd get_Kcf(const Eigen::MatrixXd& global_stiffness,
                                   const DofPartition& partition);

    static Eigen::MatrixXd get_Kcc(const Eigen::MatrixXd& global_stiffness,
                                   const DofPartition& partition);


};


#endif