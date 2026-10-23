#include "../../include/fem/Mesh.hpp"


/*********************************************************************************
                            MODIFIERS
 *********************************************************************************/
std::size_t Mesh::add_node(const double x, const double y) 
{
    const std::size_t node_id = nodes_.size();
    nodes_.push_back(Node(node_id, x, y));
    return node_id;   
}



std::size_t Mesh::add_tri3(
    std::size_t n0,
    std::size_t n1,
    std::size_t n2,
    const double thickness
) 
{
    
    if (n0 > nodes_.size())
    {
        throw 
        std::invalid_argument("First argument does not correspond to valid node id: " + n0);
    }
    if (n1 > nodes_.size())
    {
       throw 
       std::invalid_argument("Second argument does not correspond to valid node id: " + n1);
    }
    if (n2 > nodes_.size())
    {
       throw 
       std::invalid_argument("Third argument does not correspond to valid node id: " + n2);
    }
       

    const std::size_t element_id = elements_.size();
    elements_.push_back(Tri3(element_id, thickness, n0, n1, n2));
    return element_id;
}

/*********************************************************************************
                            ACCESSORS
 *********************************************************************************/
const Node& Mesh::node(const std::size_t index) const 
{
    if (index >= nodes_.size())
    {
        throw std::out_of_range("Node index is out of bounds.");
    }
    return nodes_[index];
}

const Tri3& Mesh::element(const std::size_t index) const 
{
    if (index >= elements_.size())
    {
        throw std::out_of_range("Element index is out of bounds.");
    }
    return elements_[index];
}


Mesh::const_iterator Mesh::begin() const noexcept
{
    return elements_.cbegin();
}


Mesh::const_iterator Mesh::end() const noexcept
{
    return elements_.cend();
}


std::size_t Mesh::num_nodes() const 
{
    return nodes_.size();
}


std::size_t Mesh::num_elements() const 
{
    return elements_.size();
}

/********************************************************************************
                            VALIDATION
 *********************************************************************************/
//TODO test for duplicate node indices in an element
 bool Mesh::validate(std::string& error, const double tolerance) const 
{
  
   std::ostringstream message(std::ios_base::out);
   std::vector<std::array<std::size_t, 3>> element_keys;

   element_keys.reserve(elements_.size());
   error.clear();
    //check for duplicate nodes
   for (std::size_t i = 0; i < nodes_.size(); ++i)
   {
       for (std::size_t j = i + 1; j < nodes_.size(); ++j)
       {
           if (nodes_[i].coordinates == nodes_[j].coordinates)
           {
               message << "Duplicate nodes found: " << nodes_[i] << " and " << nodes_[j] << '\n';
               error = message.str();
               return false;
           }
       }
   }

   //check for duplicate elements
   for (const Tri3& elem : elements_)
   {
        std::array<std::size_t, 3> key {
            elem.get_node_id(Tri3NodePos::first),
            elem.get_node_id(Tri3NodePos::second),
            elem.get_node_id(Tri3NodePos::third)
        };

        std::sort(key.begin(), key.end());
        element_keys.push_back(key);
   }

   std::sort(element_keys.begin(), element_keys.end());

   if (std::adjacent_find(element_keys.begin(), element_keys.end()) != element_keys.end())
   {
        error = "Duplicate elements detected.";
        return false;
   }


   //Element geometry and connectivity validation
   for (const Tri3& elem : elements_)
    {
        // Validate element connectivity before accessing nodes.

        const std::array<size_t, 3> node_ids = 
            {elem.get_node_id(Tri3NodePos::first),
             elem.get_node_id(Tri3NodePos::second),
             elem.get_node_id(Tri3NodePos::third)};

        for (auto node_id_it = node_ids.begin(); node_id_it != node_ids.end(); node_id_it++)
        {
            if (*node_id_it >= nodes_.size())
            {
                message << "Invalid connectivity in " 
                        << elem
                        << " node index: " << *node_id_it
                        << " is out of bounds. The mesh contains "
                        << nodes_.size() << " nodes.";
                error = message.str();
                return false;      
            }

            if (! Node::is_valid(node(*node_id_it)))
            {
                message << "Invalid coordinates in " 
                    << ": " << '\n';
                error = message.str();
                return false;
            }
        }



        // Check validity
        // TODO distinguish between triangles with collinear nodes and triangles with coincident nodes
        if (!Tri3::is_valid(node(node_ids[0]), node(node_ids[1]), node(node_ids[2]), tolerance))
        {
            message << "Invalid geometry in " << elem
                    << ": nodes are collinear or coincident." << '\n';
            error = message.str();
            return false;
        }

    }

    return true;
}


bool Mesh::is_valid_for_analysis(std::string& error, double tolerance) const
{
    //check for emptiness 
    //check for duplicate nodes
    //check for connnectivity & geometry validation
    //TODO
    error.clear();

    if (elements_.empty())
    {
        error = "Cannot perform analysis: the mesh contains no elements.";
        return false;
    }

    if (nodes_.empty())
    {
        error = "Cannot perform analysis: the mesh contains no nodes.";
        return false;
    }



    return validate(error, tolerance);
}