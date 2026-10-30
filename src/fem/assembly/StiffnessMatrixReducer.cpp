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


ReducedStiffnessMatrix 
StiffnessMatrixReducer::get_reduced_stiffness_matrix(
    const Eigen::MatrixXd& global_stiffness,
    const DofPartition& partition)
{
    //dynamically sized, one-dimensional array of one dimension
    using IndexArray = Eigen::Array<Eigen::Index, Eigen::Dynamic, 1>;

    if (global_stiffness.rows() != global_stiffness.cols())
        throw std::invalid_argument("Global stiffness matrix must be square.");

    IndexArray free_indices(partition.free_dofs.size()),
               constrained_indices(partition.constrained_dofs.size());

    for (std::size_t i = 0; i < partition.free_dofs.size(); ++i)
        free_indices[i] = partition.free_dofs[i];
    
    for (std::size_t i = 0; i < partition.constrained_dofs.size(); ++i)
        constrained_indices[i] = partition.constrained_dofs[i].global_dof;

     return ReducedStiffnessMatrix{
        global_stiffness(free_indices, free_indices),
        global_stiffness(free_indices, constrained_indices),
        global_stiffness(constrained_indices, free_indices),
        global_stiffness(constrained_indices, constrained_indices),
        partition
    };
}


    

