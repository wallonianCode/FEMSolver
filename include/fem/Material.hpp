#ifndef MATERIAL_H
#define MATERIAL_H

#include <cmath>
#include "Libraries.hpp"


//TODO is there any advantage in describing the arguments of a function as const 
//if these arguments are directly given to non-const variables?
struct IsotropicLinearElasticMaterial
{
    double elasticity_modulus; // E, in Pa
    double poisson_ratio;      // nu, dimensionless

    IsotropicLinearElasticMaterial(
        const double E,
        const double nu
    )

};

namespace materials
{
    inline const IsotropicLinearElasticMaterial steel{
        210e9, 0.3
    };

    inline const IsotropicLinearElasticMaterial aluminum{
        69e9, 0.33
    };
}


class LinearElastic2D 
{
protected:
	IsotropicLinearElasticMaterial material_;

public:
 	explicit LinearElastic2D(const IsotropicLinearElasticMaterial &material);

	//const makes the interface usable with const objects 
	virtual Eigen::Matrix3d get_constitutive_matrix() const = 0;

	//if derived objects might be destroyed through a base-class pointer TODO why?
	virtual ~LinearElastic2D() = default;
};



class PlaneStrain : public LinearElastic2D
{
public:
	using LinearElastic2D::LinearElastic2D; //constructor
	Eigen::Matrix3d get_constitutive_matrix() const override;
};



class PlaneStress : public LinearElastic2D
{
public:
	using LinearElastic2D::LinearElastic2D; //constructor
	Eigen::Matrix3d get_constitutive_matrix() const override;
};

#endif
