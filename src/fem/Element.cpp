#include "../../include/fem/Element.hpp"

Tri3::Tri3(const size_t id, 
           const double thickness,
           const size_t node_id1, 
           const size_t node_id2, const size_t node_id3) : id(id), thickness(thickness)
{
    node_ids = {node_id1, node_id2, node_id3};
}


bool Tri3::is_valid(const Node& node1, const Node& node2, const Node& node3, const double tolerance)
{
    return Tri3::quality(node1, node2, node3) > tolerance;
}


size_t Tri3::get_node_id(const Tri3NodePos& pos) const
{
    size_t nodeID;
    switch (pos)
    {
        case Tri3NodePos::first:
        {
            nodeID = this->node_ids[0];
            break;
        }
        case Tri3NodePos::second:
        {
            nodeID = this->node_ids[1];
            break;
        }
        case Tri3NodePos::third:
        {
            nodeID = this->node_ids[2];
            break;
        }
        default:
            break;
    }
    return nodeID;  
}


double Tri3::get_area(const Node& node1, const Node& node2, const Node& node3)
{    
    const double x1 = node1.coordinates(0);
    const double y1 = node1.coordinates(1);

    const double x2 = node2.coordinates(0);
    const double y2 = node2.coordinates(1);

    const double x3 = node3.coordinates(0);
    const double y3 = node3.coordinates(1);



    const double double_area =
        (x2 - x1) * (y3 - y1)
      - (y2 - y1) * (x3 - x1);


    return 0.5 * std::abs(double_area);
}


double Tri3::quality(const Node &node1, const Node &node2, const Node &node3)
{
    std::array<double, 3> lengths;
    const double area = Tri3::get_area(node1, node2, node3);

    const double x1 = node1.coordinates(0);
    const double y1 = node1.coordinates(1);

    const double x2 = node2.coordinates(0);
    const double y2 = node2.coordinates(1);

    const double x3 = node3.coordinates(0);
    const double y3 = node3.coordinates(1);
    

    lengths[0] = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    lengths[1] = std::sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    lengths[2] = std::sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));

    if (!std::isfinite(lengths[0]) || !std::isfinite(lengths[1]) || !std::isfinite(lengths[2]))
    {
        std::cerr << "Error: Invalid triangle geometry. Area: " << area 
                  << ", Lengths: [" << lengths[0] << ", " << lengths[1] << ", " << lengths[2] << "]" << std::endl;
        return 0.0;
    }
   
    return 2*area / std::pow(*std::max_element(lengths.begin(), lengths.end()), 2);
    //return 4.0 * std::sqrt(3.0) * area / (l1 * l1 + l2 * l2 + l3 * l3);
}


std::ostream& operator<< (std::ostream& stream, const Tri3& triangle3)
{
    stream << "Tri3 " << triangle3.id << " (Nodes: " 
           << triangle3.node_ids[0] << ", " << triangle3.node_ids[1] 
           << ", " << triangle3.node_ids[2] << ")" << '\n';
    return stream;
}
