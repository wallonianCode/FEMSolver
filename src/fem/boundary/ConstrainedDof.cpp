#include "../../../Include/fem/boundary/ConstrainedDof.hpp"


bool ConstrainedDof::operator== (const ConstrainedDof& other) const
{
    return this->global_dof == other.global_dof &&
           this->prescribed_value == other.prescribed_value;
}