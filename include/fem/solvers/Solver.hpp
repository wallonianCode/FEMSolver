#ifndef SOLVER_H
#define SOLVER_H

#include "ReactionForceCalculator.hpp"
#include "../loads/LoadConditions.hpp"
#include "../assembly/GlobalForceAssembler.hpp"

struct Result
{
    Eigen::VectorXd global_displacements;
    Eigen::VectorXd reaction_forces;

    std::vector<Eigen::Vector3d> strains;
    std::vector<Eigen::Vector3d> stresses;
};

class Solver
{
public:
    //unique_ptr: objects exclusively owns its material model 
    explicit Solver(const Mesh& mesh,
                    std::unique_ptr<LinearElastic2D> material);


    void set_boundary_conditions(const BoundaryCondition& boundary_conditions);
    void set_load_conditions(const LoadConditions& load_conditions);


    //return a struct in order to not have the user sorting out by 
    //himself the computation steps order.
    Result solve();

private:
    void assemble_global_stiffness_matrix();
    void compute_reduced_stiffness_matrix();
    void compute_reduced_system();


    Eigen::VectorXd compute_free_displacements();
    Eigen::VectorXd reconstruct_global_displacements();

    
    Eigen::VectorXd compute_reaction_forces(
        const Eigen::VectorXd& free_displacements
    );

    std::vector<Eigen::Vector3d> compute_strains(
        const Eigen::VectorXd& global_displacements
    );

    std::vector<Eigen::Vector3d> compute_stresses(
        const std::vector<Eigen::Vector3d>& strains
    );



    Mesh mesh_;

    // Input data
    std::unique_ptr<LinearElastic2D> material_;
    BoundaryCondition boundary_conditions_;
    LoadConditions load_conditions_;

    // Intermediate results
    DofManager dof_manager_;
    Eigen::MatrixXd global_stiffness_matrix_;
    Eigen::VectorXd global_force_vector_;

    ReducedStiffnessMatrix reduced_stiffness_;
    ReducedSystem reduced_system_;

    // Results
    Eigen::VectorXd global_displacements_;
    Eigen::VectorXd reaction_forces_;
};

#endif