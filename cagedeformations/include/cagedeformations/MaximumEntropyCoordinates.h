#pragma once

namespace Eigen
{
    template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    class Matrix;
    using MatrixXd = Matrix<double, -1, -1, 0, -1, -1>;
    using MatrixXi = Matrix<int, -1, -1, 0, -1, -1>;
    using Vector4d = Matrix<double, 4, 1, 0, 4, 1>;
    using Vector3d = Matrix<double, 3, 1, 0, 3, 1>;
    using VectorXd = Matrix<double, -1, 1, 0, -1, 1>;
}

#include <vector>

void calculateMaximumEntropyCoordinates(const Eigen::MatrixXd &cage_v, const Eigen::MatrixXi &cage_f, const Eigen::MatrixXd &v, Eigen::MatrixXd &mec, const int mec_flag);

// std::vector<int> findAdjacentFacesOfIndexV(const Eigen::MatrixXi &faces, int indexOfVertex);

void priorFunctions(const Eigen::Vector3d v, const Eigen::MatrixXd &cage_v, const Eigen::MatrixXi &cage_f, std::vector<std::vector<int>> adjs, Eigen::VectorXd &priors, int mec_flag);

double areaOfTriangle(const Eigen::Vector3d a, const Eigen::Vector3d b, const Eigen::Vector3d c);