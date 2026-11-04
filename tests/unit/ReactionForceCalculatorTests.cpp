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
    reduced_stiffness.Kcf = Eigen::MatrixXd::Constant(1, 1, inf);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
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

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
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

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << inf;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
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

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << nan;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
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

    reduced_stiffness.dof_partition = {
        {0},
        {{1, inf}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, InfiniteValuesInGlobalForceVector)
{
    const double inf = std::numeric_limits<double>::infinity();

    ReducedStiffnessMatrix reduced_stiffness;
    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector(2);
    global_force_vector << 0.0, inf;

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, NaNValuesInGlobalForceVector)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();

    ReducedStiffnessMatrix reduced_stiffness;
    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector(2);
    global_force_vector << 0.0, nan;

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
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

    reduced_stiffness.dof_partition = {
        {0, 1},
        {{2, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(3);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, NonSquareKcc)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 2);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(2, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Ones(2, 1);

    reduced_stiffness.dof_partition = {
        {0},
        {
            {1, 0.0},
            {2, 0.0}
        }
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(3);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, SizeMismatchKccUc)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 2);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(2, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(2, 2);

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, KcfRowsNotEqualToKccRows)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(2, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(1, 1);

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 0.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
        std::invalid_argument);
}


TEST(ReactionForceCalculatorTests, GlobalForceVectorSizeMismatch)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 2);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Ones(2, 1);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Identity(2, 2);

    reduced_stiffness.dof_partition = {
        {0},
        {
            {1, 0.0},
            {2, 0.0}
        }
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 1.0;

    // The partition describes three global DOFs, but this vector has only two.
    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    EXPECT_THROW(
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector),
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

    reduced_stiffness.dof_partition = {
        {},
        {
            {0, 1.0},
            {1, 2.0}
        }
    };

    const Eigen::VectorXd free_displacements(0);

    Eigen::VectorXd global_force_vector(2);
    global_force_vector << 1.0, 1.0;

    Eigen::VectorXd expected_reactions(2);
    expected_reactions << 1.0, 5.0;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector);

    EXPECT_TRUE(reactions.isApprox(expected_reactions));
}


TEST(ReactionForceCalculatorTests, NoPrescribedDisplacements)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(2, 2);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(2, 0);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Zero(0, 2);
    reduced_stiffness.Kcc = Eigen::MatrixXd(0, 0);

    reduced_stiffness.dof_partition = {
        {0, 1},
        {}
    };

    Eigen::VectorXd free_displacements(2);
    free_displacements << 1.0, 2.0;

    Eigen::VectorXd global_force_vector(2);
    global_force_vector << 5.0, 10.0;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector);

    EXPECT_EQ(reactions.size(), 0);
}


TEST(ReactionForceCalculatorTests, NoForcesOnConstrainedDofs)
{
    ReducedStiffnessMatrix reduced_stiffness;

    reduced_stiffness.Kff = Eigen::MatrixXd::Identity(1, 1);
    reduced_stiffness.Kfc = Eigen::MatrixXd::Zero(1, 1);
    reduced_stiffness.Kcf = Eigen::MatrixXd::Constant(1, 1, 2.0);
    reduced_stiffness.Kcc = Eigen::MatrixXd::Constant(1, 1, 3.0);

    reduced_stiffness.dof_partition = {
        {0},
        {{1, 1.0}}
    };

    Eigen::VectorXd free_displacements(1);
    free_displacements << 2.0;

    Eigen::VectorXd global_force_vector =
        Eigen::VectorXd::Zero(2);

    Eigen::VectorXd expected_reactions(1);
    expected_reactions << 7.0;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector);

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

    const Eigen::VectorXd free_displacements(0);
    const Eigen::VectorXd global_force_vector(0);

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector);

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

    reduced_stiffness.dof_partition = {
        {0, 1},
        {
            {2, 0.5},
            {3, 0.0}
        }
    };

    Eigen::VectorXd free_displacements(2);
    free_displacements << 1.0, 2.0;

    // Global DOF ordering: 0, 1, 2, 3.
    Eigen::VectorXd global_force_vector(4);
    global_force_vector << 0.0, 0.0, 1.0, 2.0;

    Eigen::VectorXd expected_reactions(2);
    expected_reactions << 5.0, 5.0;

    const Eigen::VectorXd reactions =
        ReactionForceCalculator::get_reaction_forces(
            reduced_stiffness,
            free_displacements,
            global_force_vector);

    EXPECT_TRUE(reactions.isApprox(expected_reactions));
}