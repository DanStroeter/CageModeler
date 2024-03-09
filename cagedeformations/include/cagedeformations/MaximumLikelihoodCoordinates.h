#pragma once

namespace Eigen {
    template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    class Matrix;
    using MatrixXd = Matrix<double, -1, -1, 0, -1, -1>;
    using MatrixXi = Matrix<int, -1, -1, 0, -1, -1>;
    using Vector4d = Matrix<double, 4, 1, 0, 4, 1>;
    using Vector3d = Matrix<double, 3, 1, 0, 3, 1>;
    using VectorXd = Matrix<double, -1, 1, 0, -1, 1>;
}

#include <vector>


void calculateMaximumLikelihoodCoordinates(const Eigen::MatrixXd& cage_v, const Eigen::MatrixXi& cage_f, const Eigen::MatrixXd& v, Eigen::MatrixXd& mlc);

void computeIntegralUnitNormals(const Eigen::MatrixXd& cage_v_sphere, const Eigen::MatrixXi& cage_f, Eigen::MatrixXd& transMatrix, Eigen::MatrixXd& integral_outward_allfaces);

// std::vector<int> findAdjacentFaces(const Eigen::MatrixXi &faces, int indexOfVertex);

Eigen::VectorXd f_gradient(Eigen::VectorXd x, Eigen::MatrixXd v);
Eigen::MatrixXd f_Hessian(Eigen::VectorXd x, Eigen::MatrixXd v);
double f(Eigen::VectorXd x, Eigen::MatrixXd v);
// double f(unsigned n, const double *x, double *grad, void *data);