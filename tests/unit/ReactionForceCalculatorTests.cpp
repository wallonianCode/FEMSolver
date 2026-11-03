#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

#include "../../include/fem/solvers/ReactionForceCalculator.hpp"

/** ---------------------------------------------------------------------
 *                      FINITE-VALUE VALIDATION
 *  ---------------------------------------------------------------------
 */
TEST(ReactionForceCalculatorTests, InfiniteValuesInReducedStiffnessMatrix)
{
    const double inf = std::numeric_limits<double>::infinity();

    ReducedStiffnessMatrix reduced_stiffness;
    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    //creates a 1x1 matrix initialized with inf values
    reduced_stiffness.Kcf = Eigen::MatrixXd::Constant(1, 1, inf);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << 0.0;

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, 0.0}}
    };

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, NaNValuesInReducedStiffnessMatrix)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();

    ReducedStiffnessMatrix reduced_stiffness;
    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Constant(1, 1, nan);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << 0.0;

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, 0.0}}
    };

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, InfiniteValuesInFreeDisplacements)
{
    const double inf = std::numeric_limits<double>::infinity();

    ReducedStiffnessMatrix reduced_stiffness;
    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    Eigen::VectorXd free_displacements(1);
    free_displacements << inf;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << 0.0;

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, 0.0}}
    };

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, NaNValuesInFreeDisplacements)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();

    ReducedStiffnessMatrix reduced_stiffness;
    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    Eigen::VectorXd free_displacements(1);
    free_displacements << nan;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << 0.0;

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, 0.0}}
    };

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, InfiniteValuesInPrescribedDisplacements)
{
    const double inf = std::numeric_limits<double>::infinity();

    ReducedStiffnessMatrix reduced_stiffness;
    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << 0.0;

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, inf}}
    };

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, InfiniteValuesInForcesOnConstrainedDofs)
{
    const double inf = std::numeric_limits<double>::infinity();

    ReducedStiffnessMatrix reduced_stiffness;
    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << inf;

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, 0.0}}
    };

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


/** ------------------------------------------------------------------------
 *                       DIMENSION COMPATIBILITY 
 *  ------------------------------------------------------------------------
 */

//Kcf two columns, while free displacement vector has only one coefficient
TEST(ReactionForceCalculatorTests, DimensionMismatchKcfFreeDisplacementVector)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(2, 2);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(2, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(1, 2);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    reduced_stiffness.dof_partition =
    {
        {0, 1},
        {{2, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << 0.0;

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


//Kcc 2x1 matrix
TEST(ReactionForceCalculatorTests, NonSquareKcc)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 2);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(2, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Ones(2, 1);

    reduced_stiffness.dof_partition =
    {
        {0},
        {
            {1, 0.0},
            {2, 0.0}
        }
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(2);
    forces_on_constrained_dofs << 0.0, 0.0;

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}

//Kcc 2 col - uc 1 coefficient
TEST(ReactionForceCalculatorTests, SizeMismatchKccUc)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 2);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(2, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(2, 2);

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(2);
    forces_on_constrained_dofs << 0.0, 0.0;

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, KcfRowsNotEqualToKccRows)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(2, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << 0.0;

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, ForcesOnConstrainedDofsSizeMismatchKcc)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 2);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(2, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(2, 2);

    reduced_stiffness.dof_partition =
    {
        {0},
        {
            {1, 0.0},
            {2, 0.0}
        }
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs << 0.0;

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs),
        std::invalid_argument);
}


/** ------------------------------------------------------------------------------
 *                              SPECIAL CASES
 * -------------------------------------------------------------------------------
 */


TEST(ReactionForceCalculatorTests, NoFreeDisplacements)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd(0, 0);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(0, 2);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Zero(2, 0);

    reduced_stiffness.Kcc.resize(2, 2);
    reduced_stiffness.Kcc << 2.0, 0.0,
                             0.0, 3.0;

    reduced_stiffness.dof_partition =
    {
        {},
        {
            {0, 1.0},
            {1, 2.0}
        }
    };

    const Eigen::VectorXd free_displacements;

    Eigen::VectorXd forces_on_constrained_dofs(2);
    forces_on_constrained_dofs << 1.0, 1.0;

    Eigen::VectorXd expected_reactions(2);
    expected_reactions << 1.0, 5.0;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs);

    EXPECT_TRUE(reactions.isApprox(expected_reactions));
}


TEST(ReactionForceCalculatorTests, NoPrescribedDisplacements)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(2, 2);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(2, 0);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Zero(0, 2);
    reduced_stiffness.Kcc = Eigen::MatrixXd(0, 0);

    reduced_stiffness.dof_partition =
    {
        {0, 1},
        {}
    };

    Eigen::VectorXd free_displacements(2);
    free_displacements << 1.0, 2.0;

    const Eigen::VectorXd forces_on_constrained_dofs;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs);

    EXPECT_EQ(reactions.size(), 0);
}


TEST(ReactionForceCalculatorTests, NoForcesOnConstrainedDofs)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Constant(1, 1, 2.0);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Constant(1, 1, 3.0);

    reduced_stiffness.dof_partition =
    {
        {0},
        {{1, 1.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 2.0;

    Eigen::VectorXd forces_on_constrained_dofs(1);
    forces_on_constrained_dofs.setZero();

    Eigen::VectorXd expected_reactions(1);
    expected_reactions << 7.0;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs);

    EXPECT_TRUE(reactions.isApprox(expected_reactions));
}


TEST(ReactionForceCalculatorTests, EmptyReducedStiffness)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd(0, 0);
    reduced_stiffness.Kfc = Eigen::MatrixXd(0, 0);
    reduced_stiffness.Kcf = Eigen::MatrixXd(0, 0);
    reduced_stiffness.Kcc = Eigen::MatrixXd(0, 0);

    reduced_stiffness.dof_partition = {{}, {}};

    const Eigen::VectorXd free_displacements;
    const Eigen::VectorXd forces_on_constrained_dofs;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs);

    EXPECT_EQ(reactions.size(), 0);
}


/** -------------------------------------------------------------------------------
 *                              FUNCTIONALITY
 *  -------------------------------------------------------------------------------
 */

TEST(ReactionForceCalculatorTests, GeneralCase)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(2, 2);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(2, 2);

    reduced_stiffness.Kcf.resize(2, 2);
    reduced_stiffness.Kcf << 2.0, 1.0,
                             1.0, 3.0;

    reduced_stiffness.Kcc.resize(2, 2);
    reduced_stiffness.Kcc << 4.0, 0.0,
                             0.0, 5.0;

    reduced_stiffness.dof_partition =
    {
        {0, 1},
        {
            {2, 0.5},
            {3, 0.0}
        }
    };

    Eigen::VectorXd free_displacements(2);
    free_displacements << 1.0, 2.0;

    Eigen::VectorXd forces_on_constrained_dofs(2);
    forces_on_constrained_dofs << 1.0, 2.0;

    Eigen::VectorXd expected_reactions(2);
    expected_reactions << 5.0, 5.0;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            forces_on_constrained_dofs);

    EXPECT_TRUE(reactions.isApprox(expected_reactions));
}