#include <cstddef>
#include <vector>
#include "../DofComponent.hpp"


struct NodalLoad
{
    std::size_t node_id;
    DofComponent component;
    double value;
};


class LoadConditions
{
public:
    using const_iterator =
        std::vector<NodalLoad>::const_iterator;

    void add_nodal_load(
        const std::size_t node_id,
        const DofComponent component,
        const double value);

    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

private:
    std::vector<NodalLoad> loads_;
};