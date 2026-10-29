#ifndef CONSTRAINED_DOF_H
#define CONSTRAINED_DOF_H

#include <Eigen/Dense>


struct ConstrainedDof
{
	Eigen::Index global_dof;
	double prescribed_value;
};

#endif 
