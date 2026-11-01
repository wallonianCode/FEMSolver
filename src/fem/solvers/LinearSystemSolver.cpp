 #include <Eigen/Cholesky>
#include "../../../include/fem/solvers/LinearSystemSolver.hpp"

  

Eigen::VectorXd LinearSystemSolver::solve_reduced_system(
    const ReducedSystem& reduced_system)
{
    const Eigen::MatrixXd& Kff = reduced_system.stiffness_matrix;
    const Eigen::VectorXd& F_reduced = reduced_system.force_vector;

    // Validate dimensions
    if (Kff.rows() != Kff.cols() ||
        Kff.rows() != F_reduced.size())
    {
        throw std::invalid_argument(
            "Inconsistent reduced stiffness matrix and force vector dimensions.");
    }

    // Validate numerical input.
    if (!Kff.allFinite() || !F_reduced.allFinite())
    {
        throw std::invalid_argument(
            "Reduced stiffness matrix and force vector must contain only finite values.");
    }

    // Special case: no free DOFs.
    if (Kff.rows() == 0)
        return Eigen::VectorXd(0);

    //make use of LU decomposition (cholesky) in order to avoid having to 
    //compute the inverse.
    Eigen::LLT<Eigen::MatrixXd> solver(Kff);

    if (solver.info() != Eigen::Success)
    {
        throw std::runtime_error(
            "Failed to decompose the reduced stiffness matrix.");
    }

    const Eigen::VectorXd solution = solver.solve(F_reduced);

    //checks if all coefficient of an Eigen matrix are finite.
    if (solver.info() != Eigen::Success || !solution.allFinite())
    {
        throw std::runtime_error(
            "Failed to solve the reduced linear system.");
    }

    return solution;
}