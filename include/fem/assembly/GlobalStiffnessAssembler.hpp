#ifndef GLOBAL_STIFFNESS_ASSEMBLER_H
#define GLOBAL_STIFFNESS_ASSEMBLER_H

#include "ElementStiffness.hpp"
#include "../../../include/fem/DofManager.hpp"

class GlobalStiffnessAssembler
{
  public:
    
    static Eigen::MatrixXd assemble(
        const Mesh& mesh,
        const DofManager& dof_manager,
        const LinearElastic2D& material 
    );
};

#endif
