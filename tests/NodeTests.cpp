#include <gtest/gtest.h>
#include "../include/fem/Node.hpp"

#include <limits>


TEST(NodeTests, StoresIdAndCoordinates)
{
    const Node node(3, 1.5d, -2.0d);

    EXPECT_EQ(node.id, 3);
    EXPECT_DOUBLE_EQ(node.coordinates[0], 1.5);
    EXPECT_DOUBLE_EQ(node.coordinates[1], -2.0);
}

TEST(NodeTests, AcceptsFiniteCoordinates)
{
    const Node node(0, 1.0d, 2.0d);

    EXPECT_TRUE(Node::is_valid(node));
}

TEST(NodeTests, RejectsNonFiniteCoordinates)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Node node(0, static_cast<float>(nan), 2.0f);

    EXPECT_FALSE(Node::is_valid(node));
}

TEST(NodeTests, RejectsInfiniteCoordinates)
{
    const double inf = std::numeric_limits<double>::infinity();
    const Node node(0, 1.0d, static_cast<double>(inf));

    EXPECT_FALSE(Node::is_valid(node));
}