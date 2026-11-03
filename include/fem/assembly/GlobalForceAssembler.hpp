#ifndef GLOBAL_FORCE_ASSEMBLER_H
#define GLOBAL_FORCE_ASSEMBLER_H

#include "../loads/LoadConditions.hpp"
#include "../DofManager.hpp"

struct LoadedDof
{
    Eigen::Index global_dof;
    double value;
};


class GlobalForceAssembler
{
public:
    static std::vector<LoadedDof> assemble(
        const LoadConditions& load_conditions
    );
};

#endif