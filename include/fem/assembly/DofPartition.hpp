#include <vector>
#include <Eigen\Dense>
#include "../boundary/ConstrainedDof.hpp"


struct DofPartition
{
	std::vector<Eigen::Index> free_dofs;
	std::vector<ConstrainedDof> constrained_dofs;
};