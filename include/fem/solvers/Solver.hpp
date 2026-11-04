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

//orchestration layer
class Solver
{
public:
    //user retains explicit control over mesh, mesh's lifetime exceeds 
    //that of solver. The same mesh might be given to several solvers.
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
    void compute_partition();
    void compute_global_force_vector();
    //extract the reduced stiffness blocks (Kff, Kfc...)
    void compute_reduced_stiffness_matrix();
    //computes Frel in Kffuf = Frel
    void compute_reduced_system();

    //computes uf
    void compute_free_displacements();
    void reconstruct_global_displacements();

    //computes Rc 
    void compute_reaction_forces();

    void compute_strains();
    void compute_stresses();



    Mesh mesh_;

    // Input data
    std::unique_ptr<LinearElastic2D> material_;
    BoundaryCondition boundary_conditions_;
    LoadConditions load_conditions_;

    // Intermediate results
    Eigen::MatrixXd global_stiffness_matrix_;
    Eigen::VectorXd global_force_vector_;

    DofPartition partition_;

    ReducedStiffnessMatrix reduced_stiffness_;
    ReducedSystem reduced_system_;
    Eigen::VectorXd free_displacements_;

    // Results
    Eigen::VectorXd global_displacements_;
    Eigen::VectorXd reaction_forces_;
    std::vector<Eigen::Vector3d> strains_;
    std::vector<Eigen::Vector3d> stresses_;
};

#endif