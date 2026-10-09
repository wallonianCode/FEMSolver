#include "../../include/fem/Material.hpp"

IsotropicLinearElasticMaterial::IsotropicLinearElasticMaterial(double E, double nu) :
elasticity_modulus(E), poisson_ratio(nu) 
{
    // Validate physical parameter ranges here.
    if (E <= 0.0)
        throw std::invalid_argument(
            "Elasticity modulus must be positive.");

    if (nu <= -1.0 || nu >= 0.5)
        throw std::invalid_argument(
            "Poisson ratio is outside the stable "
            "isotropic elasticity range.");
}

//-------------------------------------------  ANALYSIS TYPE --------------------------------
LinearElastic2D::LinearElastic2D(const IsotropicLinearElasticMaterial& material) : 
material_(material) {}


//------------------------------------------ PLAIN STRAIN ------------------------------------
Eigen::Matrix3d PlaneStrain::get_constitutive_matrix() const
{
    double coeff = material_.elasticity_modulus/((1 + material_.poisson_ratio) * (1 - 2*material_.poisson_ratio));
    Eigen::Matrix3d D {{1, material_.poisson_ratio, 0}, {material_.poisson_ratio, 1, 0}, {0, 0, (1-material_.poisson_ratio)/2}};
    return coeff * D;
}


//---------------------------------------- PLANE STRESS --------------------------------------
Eigen::Matrix3d PlaneStress::get_constitutive_matrix() const
{
    double coeff = material_.elasticity_modulus/(1-material_.poisson_ratio*material_.poisson_ratio);
    Eigen::Matrix3d D {{1, material_.poisson_ratio, 0}, {material_.poisson_ratio, 1, 0}, {0, 0, (1-material_.poisson_ratio)/2}};
    return coeff * D;
}


