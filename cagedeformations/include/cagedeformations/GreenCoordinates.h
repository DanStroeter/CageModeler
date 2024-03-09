#pragma once

namespace Eigen {
    template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    class Matrix;
    using MatrixXd = Matrix<double, -1, -1, 0, -1, -1>;
    using MatrixXi = Matrix<int, -1, -1, 0, -1, -1>;
    using Vector4d = Matrix<double, 4, 1, 0, 4, 1>;
}

#include <vector>

void calcNormals(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd& normals);

void calculateGreenCoordinates(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, const Eigen::MatrixXd& normals, const Eigen::MatrixXd& eta_m,
	Eigen::MatrixXd& phi, Eigen::MatrixXd& psi);
void calculateGreenCoordinatesFromQMVC(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, const Eigen::MatrixXd& normals, const Eigen::MatrixXd& eta_m,
    Eigen::MatrixXd& phi, Eigen::MatrixXd& psi);

void calculateGreenCoordinatesTriQuad(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd const& eta_m,
    Eigen::MatrixXd& phi, std::vector<double>& psi_tri, std::vector<Eigen::Vector4d>& psi_quad);

void calcNewPositionsTriQuad(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed, const Eigen::MatrixXi& CF,
    const Eigen::MatrixXd& phi, std::vector<double> const& psi_tri, std::vector<Eigen::Vector4d> const& psi_quad, Eigen::MatrixXd& eta_deformed);

void calcScalingFactors(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed, const Eigen::MatrixXi& CF, Eigen::MatrixXd& normals);

void calcMVC(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd const& eta_m,
    Eigen::MatrixXd& phi);

bool computeMVCTriQuad(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd const& eta_m,
    Eigen::MatrixXd& phi);
