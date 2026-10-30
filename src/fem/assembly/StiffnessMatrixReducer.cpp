#include "../../../include/fem/assembly/StiffnessMatrixReducer.hpp"


DofPartition StiffnessMatrixReducer::partition_dofs(
		const Eigen::Index total_dofs,
		const std::vector<ConstrainedDof>& constrained_dofs
	)
{
    DofPartition partition;
    partition.constrained_dofs = constrained_dofs;

    std::vector<Eigen::Index> all_dofs(total_dofs);
    std::iota(all_dofs.begin(), all_dofs.end(), 0);

    std::vector<Eigen::Index> constrained_indices;
    constrained_indices.reserve(constrained_dofs.size());

    for (const ConstrainedDof& dof : constrained_dofs)
        constrained_indices.push_back(dof.global_dof);

    std::sort(constrained_indices.begin(), constrained_indices.end());

    //difference between first range & second range stored into range beginning at iterator
    //in this case, we define a back iterator (instead of a forward for example) on the partition range,
    //in order to avoid to have to allocate a vector beforehand (this iterator calls push_back() on every increment)
    std::set_difference(
        all_dofs.begin(), all_dofs.end(),
        constrained_indices.begin(), constrained_indices.end(),
        std::back_inserter(partition.free_dofs)
    );

    return partition;
}

Eigen::MatrixXd StiffnessMatrixReducer::get_Kff(const Eigen::MatrixXd& global_stiffness,
                                                const DofPartition& partition) 
{
    //TODO
}

Eigen::MatrixXd StiffnessMatrixReducer::get_Kfc(const Eigen::MatrixXd& global_stiffness,
                                                const DofPartition& partition)
{
    //TODO
}
    
Eigen::MatrixXd StiffnessMatrixReducer::get_Kcf(const Eigen::MatrixXd& global_stiffness,
                                                const DofPartition& partition)
{
    //TODO
}

Eigen::MatrixXd StiffnessMatrixReducer::get_Kcc(const Eigen::MatrixXd& global_stiffness,
                                                const DofPartition& partition)
{
    //TODO
}