#ifndef _COMPUTING_H_
#define _COMPUTING_H_

#include <cstdint>

#include "Eigen/Dense"
namespace Computing {

	template<typename _ComputingType, uint32_t _Rows, uint32_t _Cols>
	class Matrix {
	public:
		Matrix();
		Matrix(double x, double y, double z);

		_ComputingType norm();
		void normalize();
		_ComputingType& operator()(uint32_t x, uint32_t y);
	};

	using Vector3d = Matrix<double, 3, 1>;
	using Matrix3d = Matrix<double, 3, 3>;


	template<typename _ComputingType>
	class AngleAxis {
	public:
		AngleAxis(_ComputingType norm, Matrix<_ComputingType, 3, 1> vec);
		Matrix<_ComputingType, 3, 3> matrix();
	};

	using AngleAxisd = AngleAxis<double>;
};

#endif // !_COMPUTING_H_

