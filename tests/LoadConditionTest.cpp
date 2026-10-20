#include <gtest/gtest.h>
#include "../include/fem/loads/LoadConditions.hpp"


TEST(LoadConditionsTest, AddNodalLoad)
{
    LoadConditions lc;
    lc.add_nodal_load(1, DofComponent::X, 0.5);
    lc.add_nodal_load(2, DofComponent::Y, 1.0);

    auto it = lc.begin();
    EXPECT_EQ(it->node_id, 1);
    EXPECT_EQ(it->component, DofComponent::X);
    //allows a small difference of up to 4 units in the last place (ULP) between the two values
    //doubles are not always exactly representable in binary.
    EXPECT_DOUBLE_EQ(it->value, 0.5);

    ++it;
    EXPECT_EQ(it->node_id, 2);
    EXPECT_EQ(it->component, DofComponent::Y);
    EXPECT_DOUBLE_EQ(it->value, 1.0);

    ++it;
    EXPECT_EQ(it, lc.end());
}


TEST(LoadConditionsTest, InitiallyEmpty)
{
    const LoadConditions lc;
    EXPECT_EQ(lc.begin(), lc.end());
}