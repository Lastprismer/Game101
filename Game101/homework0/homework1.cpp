#include "homework1.h"

int homework1()
{
	using namespace Eigen;
	Vector3f p(2, 1, 1);
	Matrix<float, 3, 3> rotation, transform;
	float cos = sqrt(2) / 2;
	rotation <<
		cos, -cos, 0,
		cos, cos, 0,
		0, 0, 1;
	transform <<
		1, 0, 1,
		0, 1, 2,
		0, 0, 1;
	auto k = transform * rotation * p;
	std::cout << k;
	return 0;
}