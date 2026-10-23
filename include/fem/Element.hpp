#ifndef ELEMENT_H
#define ELEMENT_H

#include <array>
#include <algorithm>
#include <cmath>
#include "../../include/fem/Node.hpp"

enum class Tri3NodePos
{
	first, 
	second,
	third
};

struct Tri3 {
	std::size_t id;
	double thickness;
	//store the ids only instead of copies of the nodes.
	//Since the mesh is going to own all the nodes, the nodes need to be updated there.


	Tri3(const size_t id, const double thickness, 
		 const size_t node_id1, const size_t node_id2, const size_t node_id3);
	static bool is_valid(const Node &node1, const Node &node2, const Node &node3, const double tolerance);


	friend std::ostream& operator<< (std::ostream& stream, const Tri3& triangle3);

	//get first at pos iPos 
	size_t get_node_id(const Tri3NodePos& pos) const;
	static double get_area(const Node &node1, const Node &node2, const Node &node3);
  private:
  	std::array<std::size_t, 3> node_ids; 
	static double quality(const Node &node1, const Node &node2, const Node &node3);	
};


std::ostream& operator<< (std::ostream& stream, const Tri3& triangle3);
#endif
