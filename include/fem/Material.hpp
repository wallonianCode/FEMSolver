#ifndef MATERIAL_H
#define MATERIAL_H

#include <cmath>

struct Steel
{
	inline static const double E = 210*std::pow(10, 9);
	inline static const float v = 0.3;
};

#endif
