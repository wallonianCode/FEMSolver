#include <limits>
#include "../../include/fem/DofManager.hpp"


Eigen::Vector2i DofManager::get_dofs_from_node(
	const Node& node)
    {
		int first_dof;
        // Ensure the resulting indices fit in int.
        constexpr auto max_id =
            static_cast<std::size_t>(
                (std::numeric_limits<int>::max() - 1) / 2);

        if (node.id > max_id)
        {
            throw std::overflow_error(
                "Node ID is too large for DOF indices.");
        }

        first_dof = 
			2 * static_cast<int>(node.id);

        return Eigen::Vector2i(
            first_dof,
            first_dof + 1
        );
    }

    
Eigen::Vector2i DofManager::get_dofs_from_node_id(
    const size_t& node_id
)
{
    int first_dof;
    // Ensure the resulting indices fit in int.
    constexpr auto max_id =
        static_cast<std::size_t>(
            (std::numeric_limits<int>::max() - 1) / 2);

    if (node_id > max_id)
    {
        throw std::overflow_error(
            "Node ID is too large for DOF indices.");
    }

    first_dof = 
        2 * static_cast<int>(node_id);

    return Eigen::Vector2i(
        first_dof,
        first_dof + 1
    );

}
