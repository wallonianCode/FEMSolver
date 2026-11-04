#ifndef GLOBAL_FORCE_ASSEMBLER_H
#define GLOBAL_FORCE_ASSEMBLER_H

#include "../loads/LoadConditions.hpp"
#include "../DofManager.hpp"
#include "../Mesh.hpp"



class GlobalForceAssembler
{
public:
    static Eigen::VectorXd assemble(
        const Mesh& mesh,
        const LoadConditions& load_conditions
    );

};

#endif