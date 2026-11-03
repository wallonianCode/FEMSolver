#include "../../../include/fem/solvers/Solver.hpp"
#include "../../../BoundaryConditionsHandler.hpp"


Solver::Solver(const Mesh& mesh, 
               std::unique_ptr<LinearElastic2D> material) : 
               mesh_(mesh), material_(std::move(material)) 
{
    if (!material_)
    {
        throw std::invalid_argument("Material must not be null");
    }
}


void Solver::set_boundary_conditions(const BoundaryCondition& boundary_conditions) : 
    boundary_conditions_(boundary_conditions) {}


void Solver::set_load_conditions(const LoadConditions& load_conditions) :
    load_conditions_(load_conditions) {}


Result Solver::solve()
{
    assemble_global_stiffness_matrix();
    compute_reduced_stiffness_matrix();
    compute_reduced_system();

    const Eigen::VectorXd free_displacements =
        compute_free_displacements();

    const Eigen::VectorXd global_displacements =
        reconstruct_global_displacements();

    const Eigen::VectorXd reaction_forces =
        compute_reaction_forces(free_displacements);

    const auto strains =
        compute_strains(global_displacements);

    const auto stresses =
        compute_stresses(strains);

    return {
        global_displacements,
        reaction_forces,
        strains,
        stresses
    };
}



void Solver::assemble_global_stiffness_matrix() 
{ 
    this->global_stiffness_matrix_ = 
    GlobalStiffnessAssembler::assemble(
        this->mesh_,
        *this->material_);
}

 
void Solver::compute_reduced_stiffness_matrix()
{
    const DofPartition partition = 
        StiffnessMatrixReducer::partition_dofs(
            this->mesh_.num_nodes()*2,
            BoundaryConditionsHandler::get_constrained_dofs_vector(
                this->boundary_conditions_
            )
        );
    
    this->reduced_stiffness_ = 
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            this->global_stiffness_matrix_,
            partition
        );
}


void Solver::compute_reduced_system()
{
    this->reduced_system_ =
        LinearSystemReducer::get_free_displacement_system(
            this->reduced_stiffness_,
            //need global forces assembler
        )
    //TODO
}




Eigen::VectorXd Solver::compute_free_displacements()
{
    //TODO
}


Eigen::VectorXd Solver::compute_reaction_forces(
    const Eigen::VectorXd& free_displacements
)
{
    //TODO
}