#ifndef LOAD_CONDITIONS_HANDLER
#define LOAD_CONDITIONS_HANDLER

#include "LoadConditions.hpp"
#include "LoadedDof.hpp"
#include "../DofManager.hpp"

class LoadConditionsHandler
{
public:
    static std::vector<LoadedDof> 
        get_loaded_dofs_vector(const LoadConditions& load_conditions);
};

#endif