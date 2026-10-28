#ifndef ELEMENT_STIFFNESS_HPP
#define ELEMENT_STIFFNESS_HPP

#include <cmath>
#include <stdexcept>


#include "../Element.hpp"
#include "../Material.hpp"
#include "../Mesh.hpp"



class ElementStiffness
{
  public:
    // type aliases
    using Matrix3d = Eigen::Matrix3d;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix36d = Eigen::Matrix<double, 3, 6>;

    static Matrix6d get_stiffness_matrix(const Tri3& element,
                                         const Mesh& mesh,
                                         const LinearElastic2D& material);
  
  private:
    static const Matrix36d calculate_B(const Tri3& element, 
                                       const Mesh& mesh);

    static const Matrix3d calculate_D(const LinearElastic2D& material);

    static double calculate_area(const Tri3& Element,
                                 const Mesh& mesh);
};

#endif
