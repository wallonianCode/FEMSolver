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