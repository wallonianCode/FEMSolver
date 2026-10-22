#include "../../../include/fem/assembly/ElementStiffness.hpp"



ElementStiffness::Matrix6d ElementStiffness::get_stiffness_matrix(
    const Tri3& element,
    const Mesh& mesh,
    const LinearElastic2D& material)
{
   const ElementStiffness::Matrix36d B = 
        ElementStiffness::calculate_B(element, mesh);

   return element.thickness * 
          ElementStiffness::calculate_area(element, mesh) *
          B.transpose() *
          ElementStiffness::calculate_D(material) *
          B;      
}
  

const ElementStiffness::Matrix36d ElementStiffness::calculate_B(
    const Tri3& element, 
    const Mesh& mesh)
{
    // Retrieve the three nodes.
    const Node& n1 =
        mesh.node(element.get_node_id(Tri3NodePos::first));

    const Node& n2 =
        mesh.node(element.get_node_id(Tri3NodePos::second));

    const Node& n3 =
        mesh.node(element.get_node_id(Tri3NodePos::third));

    // Extract global coordinates.
    const double x1 = n1.coordinates.x();
    const double y1 = n1.coordinates.y();

    const double x2 = n2.coordinates.x();
    const double y2 = n2.coordinates.y();

    const double x3 = n3.coordinates.x();
    const double y3 = n3.coordinates.y();

    // Twice the signed area.
    const double detJ =
        (x2 - x1) * (y3 - y1)
      - (x3 - x1) * (y2 - y1);

    if (!std::isfinite(detJ) || detJ == 0.0)
    {
        throw std::domain_error(
            "Cannot calculate B for a degenerate triangle.");
    }

    // Shape-function derivatives.
    const double b1 = y2 - y3;
    const double b2 = y3 - y1;
    const double b3 = y1 - y2;

    const double c1 = x3 - x2;
    const double c2 = x1 - x3;
    const double c3 = x2 - x1;

    Matrix36d B;

    B <<
        b1, 0.0, b2, 0.0, b3, 0.0,
        0.0, c1, 0.0, c2, 0.0, c3,
        c1, b1, c2, b2, c3, b3;

    B /= std::abs(detJ);

    return B;
}


const ElementStiffness::Matrix3d ElementStiffness::calculate_D(
    const LinearElastic2D& material)
{
    return material.get_constitutive_matrix();
}


static const double calculate_area(const Tri3& element,
                                   const Mesh& mesh)
{
    return element.get_area(mesh.node(element.get_node_id(Tri3NodePos::first)),
                            mesh.node(element.get_node_id(Tri3NodePos::second)),
                            mesh.node(element.get_node_id(Tri3NodePos::third)));
}