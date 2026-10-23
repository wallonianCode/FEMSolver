#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include  "../include/fem/boundary/BoundaryCondition.hpp"


TEST(BoundaryConditionTest, AddDisplacement)
{
    BoundaryCondition bc;
    bc.add_displacement(1, DofComponent::X, 0.5);
    bc.add_displacement(2, DofComponent::Y, 1.0);

    auto it = bc.begin();
    EXPECT_EQ(it->node_id, 1);
    EXPECT_EQ(it->component, DofComponent::X);
    EXPECT_DOUBLE_EQ(it->value, 0.5);

    ++it;
    EXPECT_EQ(it->node_id, 2);
    EXPECT_EQ(it->component, DofComponent::Y);
    EXPECT_DOUBLE_EQ(it->value, 1.0);

    ++it;
    EXPECT_EQ(it, bc.end());
}


TEST(BoundaryConditionTest, InitiallyEmpty)
{
    const BoundaryCondition bc;
    EXPECT_EQ(bc.begin(), bc.end());
}


TEST(BoundaryConditionTest, RejectsNonFiniteDisplacement)
{
    BoundaryCondition bc;

    //undefined / unrepresentable result
    EXPECT_THROW(
        bc.add_displacement(
            0, DofComponent::X,
            std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);

    //value larger than any representable finite value
    EXPECT_THROW(
        bc.add_displacement(
            0, DofComponent::Y,
            std::numeric_limits<double>::infinity()),
        std::invalid_argument);
}
