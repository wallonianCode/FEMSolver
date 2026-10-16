#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "../include/fem/Node.hpp"
#include "../include/fem/Element.hpp"
#include "../include/fem/Mesh.hpp"

const double VERY_LOW_QUALITY = 0.05;
const double LOW_QUALITY = 0.1;
const double HIGH_QUALITY = 0.35;
const double VERY_HIGH_QUALITY = 0.5;

TEST(MeshTests, CanAddNodesAndElements)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n2);
    EXPECT_EQ(mesh.num_nodes(), 3);
    EXPECT_EQ(mesh.num_elements(), 1);
}

//Access element with valid node index
TEST(MeshTests, AccessElementWithValidNodeIndex)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n2);
    const Tri3& retrieved_element = mesh.element(0);
    EXPECT_EQ(retrieved_element.node_ids[0], n0);
    EXPECT_EQ(retrieved_element.node_ids[1], n1);
    EXPECT_EQ(retrieved_element.node_ids[2], n2);
}


TEST(MeshTests, AccessElementWithInvalidNodeIndex)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n2);
    EXPECT_THROW(mesh.element(1), std::out_of_range);
}


TEST(MeshTests, ValidateMeshWithValidElements)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n2);
    std::string error;
    EXPECT_TRUE(mesh.validate(error, LOW_QUALITY));
}


TEST(MeshTests, ValidateMeshWithInvalidElements)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 1.0f);
    const std::size_t n2 = mesh.add_node(2.0f, 2.0f); // Colinear nodes
    mesh.add_tri3(n0, n1, n2);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("coordinates"));
}


TEST(MeshTests, ValidateMeshWithDuplicateNodes)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(1.0f, 0.0f); // Duplicate node
    mesh.add_tri3(n0, n1, n2);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("Duplicate"));
}


TEST(MeshTests, ValidateMeshWithDuplicateElements)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n2);
    mesh.add_tri3(n0, n1, n2);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("Duplicate"));
}


TEST(MeshTests, ValidateMeshWithInvalidNodeCoordinates)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(std::numeric_limits<double>::infinity(), 0.0f); // Invalid coordinate
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n2);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("coordinates"));
}


TEST(MeshTests, ValidateMeshWithInvalidElementQuality)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(1.0f, 1e-6f); // Very narrow triangle
    mesh.add_tri3(n0, n1, n2);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("geometry"));
}


TEST(MeshTests, ValidateMeshWithMultipleElements)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    const std::size_t n3 = mesh.add_node(1.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n2);
    mesh.add_tri3(n1, n3, n2);
    std::string error;
    EXPECT_TRUE(mesh.validate(error, LOW_QUALITY));
    EXPECT_TRUE(error.empty());
}


TEST(MeshTests, ValidateMeshWithInvalidElementInMultipleElements)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    const std::size_t n3 = mesh.add_node(1.5f, std::sqrt(3.0) / 2.0);
    const std::size_t n4 = mesh.add_node(2.5f, std::sqrt(3.0) / 2.0); // Invalid node for the second triangle
    mesh.add_tri3(n0, n1, n2);
    mesh.add_tri3(n2, n4, n3);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("geometry"));
}


TEST(MeshTests, ValidateMeshWithInvalidNodeIndexInElement)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, 999); // Invalid node index
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("connectivity"));
}


TEST(MeshTests, ValidateMeshWithDuplicateNodeIndicesInElement)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n1);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("Duplicate"));
}


TEST(MeshTests, ValidateMeshWithInvalidNodeCoordinatesInElement)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(std::numeric_limits<double>::infinity(), 0.0f); // Invalid coordinate
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    mesh.add_tri3(n0, n1, n2);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("coordinates"));
}


TEST(MeshTests, ValidateMeshWithInvalidElementQualityInMultipleElements)
{
    Mesh mesh;
    const std::size_t n0 = mesh.add_node(0.0f, 0.0f);
    const std::size_t n1 = mesh.add_node(1.0f, 0.0f);
    const std::size_t n2 = mesh.add_node(0.5f, std::sqrt(3.0) / 2.0);
    const std::size_t n3 = mesh.add_node(1.5f, std::sqrt(3.0) / 2.0);
    const std::size_t n4 = mesh.add_node(1.5f, 1e-6f); // Very narrow triangle
    mesh.add_tri3(n0, n1, n2);
    mesh.add_tri3(n1, n4, n3);
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("geometry"));
}


TEST(MeshTests, ValidateMeshWithNoNodes)
{
    Mesh mesh;
    std::string error;
    EXPECT_FALSE(mesh.validate(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("no nodes"));
}


TEST(MeshTests, ValidateMeshWithoutElements)
{
    Mesh mesh;
    std::string error;
    EXPECT_FALSE(mesh.is_valid_for_analysis(error, LOW_QUALITY));
    EXPECT_FALSE(error.empty());
    EXPECT_THAT(error, ::testing::HasSubstr("no elements"));
}