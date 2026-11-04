#ifndef LOADED_DOF_H
#define LOADED_DOF_H

#include <Eigen\Dense>



struct LoadedDof
{
    Eigen::Index global_dof;
    double value;

    bool operator==(const LoadedDof& other) const;
};

#endif