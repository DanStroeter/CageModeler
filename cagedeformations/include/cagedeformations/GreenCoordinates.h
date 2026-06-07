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

void computeMVC(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd const& eta_m,
    Eigen::MatrixXd& phi);

bool computeMVCTriQuad(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd const& eta_m,
    Eigen::MatrixXd& phi);

//BGC related code begin
void calcNewPositionsBezier(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed, const Eigen::MatrixXi& CF, int dim, std::vector<Eigen::MatrixXd>& patch_Bezier_normals_original, std::vector<Eigen::MatrixXd>& patch_Bezier_normals,
    const Eigen::MatrixXd& phi, std::vector<Eigen::MatrixXd> const& psi_bezier, Eigen::MatrixXd& eta_deformed, std::vector<int>& num_vertices_per_line);
void calcNewPositionsBezierCrossProduct(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed, const Eigen::MatrixXi& CF, int dim, std::vector<Eigen::MatrixXd>& patch_Bezier_cross_original, std::vector<Eigen::MatrixXd>& patch_Bezier_cross,
    const Eigen::MatrixXd& phi, std::vector<Eigen::MatrixXd> const& psi_bezier, Eigen::MatrixXd& eta_deformed, std::vector<int>& num_vertices_per_line);
void calculateGreenCoordinatesBezier(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, int dim, std::vector<Eigen::MatrixXd>& patch_Bezier_normals, Eigen::MatrixXd const& eta_m,
    Eigen::MatrixXd& phi_bezier, std::vector<Eigen::MatrixXd>& psi_bezier, std::vector<int>& num_vertices_per_line);
void calculateGreenCoordinatesBezierCrossProduct(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, int dim, std::vector<Eigen::MatrixXd>& patch_Bezier_normals, Eigen::MatrixXd const& eta_m,
    Eigen::MatrixXd& phi_bezier, std::vector<Eigen::MatrixXd>& psi_bezier, std::vector<int>& num_vertices_per_line);
//BGC related code end
