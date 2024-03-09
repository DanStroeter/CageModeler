#ifndef NUMERIC_DEF_H
#define NUMERIC_DEF_H

#include <Eigen/Dense>

enum DeformerType {
  SOMIGLIANA,
  GREEN,
  MEANVALUE,
  PHONG
};

enum BulgingType {
  SOLID_ANGLE,
  SWEPT_VOLUME
};

typedef std::complex<double> complex_t;
typedef Eigen::Matrix<complex_t, 3, 3> Matrix3c;
typedef Eigen::Matrix<complex_t, 9, 3> Matrix93c;
typedef Eigen::Matrix<double, 9, 3> Matrix93d;

typedef Eigen::Matrix<double, 81, 1> Vector81d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 24, 24> Matrix24d;
typedef Eigen::Matrix<double, 3, 8> Matrix3x8d;
typedef Eigen::Matrix<double, 8, 3> Matrix8x3d;

typedef Eigen::Matrix<size_t, -1, -1> mati_t;
typedef Eigen::Matrix<double, -1, -1> matd_t;
typedef Eigen::Matrix<size_t, -1,  1> veci_t;
typedef Eigen::Matrix<double, -1,  1> vecd_t;

#endif // NUMERIC_DEF_H
