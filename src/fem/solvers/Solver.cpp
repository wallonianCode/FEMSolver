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


void Solver::set_boundary_conditions(const BoundaryCondition& boundary_conditions)
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
     * TODO introduce try/catch block for exceptions. If the process is interrupted,
     * we need to empty the intermediate results
     */
    if (mesh_.num_nodes() == 0 || mesh_.num_elements() == 0)
    {
        throw std::invalid_argument(
            "Cannot solve a mesh without nodes and elements."
        );
    }

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
    //for each element of the mesh, extract the 
    //corresponding nodal displacements out of the 
    //vector of global displacements.
    this->strains_.clear();
    for (const Tri3& elem: this->mesh_)
    {
      const size_t n1 = elem.get_node_id(Tri3NodePos::first);
      const size_t n2 = elem.get_node_id(Tri3NodePos::second);
      const size_t n3 = elem.get_node_id(Tri3NodePos::third);
      
      const Eigen::Vector2i dof1 =
        DofManager::get_dofs_from_node_id(n1);
      const Eigen::Vector2i dof2 =
        DofManager::get_dofs_from_node_id(n2);
      const Eigen::Vector2i dof3 =
        DofManager::get_dofs_from_node_id(n3);

      Eigen::Matrix<double, 6, 1> ue;
      ue << 
          this->global_displacements_(dof1[0]),
          this->global_displacements_(dof1[1]),
          this->global_displacements_(dof2[0]),
          this->global_displacements_(dof2[1]),
          this->global_displacements_(dof3[0]),
          this->global_displacements_(dof3[1]);
      //multiply this vector of nodal displacements 
      //by the B matrix of the StiffnessElement class
      this->strains_.push_back(
        ElementStiffness::calculate_B(elem, this->mesh_) * 
        ue
       );
    }
}


void Solver::compute_stresses()
{
    if (this->strains_.size() != 
        this->mesh_.num_elements())
    {
        throw(std::runtime_error(
            "Strain vector does not contain one entry per element."
        ));
    }
    this->stresses_.clear();

    //we assume that each element shares the same material
    const Eigen::MatrixXd D =  
        ElementStiffness::calculate_D(*(this->material_));
    for (const Eigen::Vector3d& strain : this->strains_)
    {
      this->stresses_.push_back(D * strain);
    }
}