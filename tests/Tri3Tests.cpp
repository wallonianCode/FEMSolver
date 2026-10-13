#include <gtest/gtest.h>
#include "../include/fem/Node.hpp"
#include "../include/fem/Element.hpp"

const double VERY_LOW_QUALITY = 0.05;
const double LOW_QUALITY = 0.1;
const double HIGH_QUALITY = 0.35;
const double VERY_HIGH_QUALITY = 0.5;


TEST(Tri3Tests, ValidityOfRightTriangle)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 2.0f, 0.0f);
    const Node n2(2, 0.0f, 1.0f);
    EXPECT_TRUE(Tri3::is_valid(n0, n1, n2, HIGH_QUALITY));
}


TEST(Tri3Tests, ValidityOfEquilateralTriangle)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 1.0f, 0.0f);
    const Node n2(2, 0.5f, std::sqrt(3.0) / 2.0);
    EXPECT_TRUE(Tri3::is_valid(n0, n1, n2, VERY_HIGH_QUALITY));
}


TEST(Tri3Tests, ValidityOfClockwiseNodeOrder)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 2.0f, 0.0f);
    const Node n2(2, 0.0f, 1.0f);
    EXPECT_TRUE(Tri3::is_valid(n1, n2, n0, LOW_QUALITY));
}


TEST(Tri3Tests, ValidityOfCounterClockwiseNodeOrder)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 2.0f, 0.0f);
    const Node n2(2, 0.0f, 1.0f);
    EXPECT_TRUE(Tri3::is_valid(n0, n2, n1, LOW_QUALITY));
}


TEST(Tri3Tests, ThreeColinearNodesProduceInvalidTriangle)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 1.0f, 1.0f);
    const Node n2(2, 2.0f, 2.0f);
    EXPECT_FALSE(Tri3::is_valid(n0, n1, n2, VERY_LOW_QUALITY));
}


TEST(Tri3Tests, TwoIdenticalNodesProduceInvalidTriangle)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 1.0f, 1.0f);
    const Node n2(2, 1.0f, 1.0f);
    EXPECT_FALSE(Tri3::is_valid(n0, n1, n2, VERY_LOW_QUALITY));
}


TEST(Tri3Tests, VeryNarrowTriangleIsInvalidForSmallQUALITY)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 1.0f, 0.0f);
    const Node n2(2, 1.0f, 1e-6f);
    EXPECT_FALSE(Tri3::is_valid(n0, n1, n2, VERY_LOW_QUALITY));
}


TEST(Tri3Tests, EquilateralTriangleIsValidForHighQUALITY)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 1.0f, 0.0f);
    const Node n2(2, 0.5f, std::sqrt(3.0) / 2.0);
    EXPECT_TRUE(Tri3::is_valid(n0, n1, n2, VERY_HIGH_QUALITY));
}



TEST(Tri3Tests, SameTriangleScaledAreValidForSameQUALITY)
{
    const Node n0(0, 0.0f, 0.0f);
    const Node n1(1, 1.0f, 0.0f);
    const Node n2(2, 0.5f, std::sqrt(3.0) / 2.0);

    const Node n3(3, 0.0f, 0.0f);
    const Node n4(4, 2.0f, 0.0f);
    const Node n5(5, 1.0f, std::sqrt(3.0));

    EXPECT_TRUE(Tri3::is_valid(n0, n1, n2, HIGH_QUALITY));
    EXPECT_TRUE(Tri3::is_valid(n3, n4, n5, HIGH_QUALITY));
}