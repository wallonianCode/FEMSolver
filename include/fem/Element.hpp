#ifndef ELEMENT_H
#define ELEMENT_H

#include <array>


struct Tri3 {
	std::size_t id;
	//store the ids only instead of copies of the nodes.
	//Since the mesh is going to own all the nodes, the nodes need to be updated there.
	std::array<std::size_t, 3> node_ids; 
private:

};

#endif
