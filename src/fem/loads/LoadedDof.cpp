#include "../../../include/fem/loads/LoadedDof.hpp"


bool LoadedDof::operator==(const LoadedDof& other) const
{
    return this->global_dof == other.global_dof;
}