#include "../../../include/fem/solvers/Solver.hpp"
#include "../../../include/fem/boundary/BoundaryConditionsHandler.hpp"


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
{
    this->boundary_conditions_ = boundary_conditions;
}


void Solver::set_load_conditions(const LoadConditions& load_conditions) 
{
    this->load_conditions_ = load_conditions;
}


Result Solver::solve()
{
    /**
     * TODO
     * There is, however, one consideration: if a calculation throws an exception halfway through solve(), some members may contain newly calculated values while others still contain values from a previous run.

For now, I wouldn't introduce a more complicated architecture just to address this. But eventually, you could consider keeping intermediate results local to solve() and only updating the final result after successful completion.
     */
    assemble_global_stiffness_matrix();
    compute_partition();
    compute_global_force_vector();

    compute_reduced_stiffness_matrix();
    compute_reduced_system();
    compute_free_displacements();
    
    reconstruct_global_displacements();
    compute_reaction_forces();
    compute_strains();
    compute_stresses();

    return {
        global_displacements_,
        reaction_forces_,
        strains_,
        stresses_
    };
}



void Solver::assemble_global_stiffness_matrix() 
{ 
    this->global_stiffness_matrix_ = 
    GlobalStiffnessAssembler::assemble(
        this->mesh_,
        *this->material_);
}


void Solver::compute_partition()
{
    this->partition_ = 
        StiffnessMatrixReducer::partition_dofs(
            this->mesh_.num_nodes()*2,
            BoundaryConditionsHandler::get_constrained_dofs_vector(
                this->boundary_conditions_
            )
        );
}

void Solver::compute_global_force_vector()
{
    this->global_force_vector_ =
        GlobalForceAssembler::assemble(
            this->mesh_,
            this->load_conditions_
        );
}


void Solver::compute_reduced_stiffness_matrix()
{
    this->reduced_stiffness_ = 
        StiffnessMatrixReducer::get_reduced_stiffness_matrix(
            this->global_stiffness_matrix_,
            this->partition_
        );
}


void Solver::compute_reduced_system()
{
    this->reduced_system_ =
        LinearSystemReducer::get_free_displacement_system(
            this->reduced_stiffness_,
            this->global_force_vector_
        );
}


void Solver::compute_free_displacements()
{
    this->free_displacements_ = 
        LinearSystemSolver::solve_reduced_system(this->reduced_system_); 
}

void Solver::reconstruct_global_displacements()
{
    this->global_displacements_ =
        GlobalDisplacementsReconstructor::get_global_displacements_vector(
            this->free_displacements_,
            this->partition_
        );
}


void Solver::compute_reaction_forces()
{
    this->reaction_forces_ = 
        ReactionForceCalculator::get_reaction_forces(
            this->reduced_stiffness_,
            this->free_displacements_,
            this->global_force_vector_
        );
}


void Solver::compute_strains() 
{
    //e = du/dx
    //TODO
}


void Solver::compute_stresses()
{
    //TODO
}