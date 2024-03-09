#ifndef UTIL_H
#define UTIL_H

#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <somigliana/types.h>

namespace green {

extern "C" {
  void ld_by_order_(int *order, double *x, double *y, double *z, double *w);
  void xyz_to_tp_(double *x, double *y, double *z, double *phi, double *theta);
}

inline bool isAtOrigin(const double square_r) {
  return square_r < 1e-18;
}

inline void xyz_to_pt(const double &&x, const double &&y, const double &&z,
                             double *phi, double *theta) {
  xyz_to_tp_(const_cast<double*>(&x),
             const_cast<double*>(&y),
             const_cast<double*>(&z),
             phi, theta);
}

inline void xyz_to_rpt(const double &x, const double &y, const double &z,
                       double &r, double &phi, double &theta) {
  const double R2 = x*x+y*y+z*z;
  if ( isAtOrigin(R2) ) {
    r = phi = theta = 0; // phi and theta can be arbitrary
  } else {
    r = std::sqrt(R2);
    xyz_to_pt(x/r, y/r, z/r, &phi, &theta);
  }
}

inline double ArcCsch(const double x) { // only handle x > 0
  return std::log((1.0+sqrt(1.0+x*x))/x);
}

inline int rank4Index(const int i, const int j, const int k, const int l) {
  return l+3*k+3*3*j+3*3*3*i;
}

inline int voigt_map(const int i, const int j) {
  if ( i == j ) {
    return i;
  }
  if ( (i == 1 && j == 2) || (i == 2 && j == 1) ) {
    return 3;
  }
  if ( (i == 0 && j == 2) || (i == 2 && j == 0) ) {
    return 4;
  }
  if ( (i == 0 && j == 1) || (i == 1 && j == 0) ) {
    return 5;
  }
  return -1;
}

inline Vector81d convertNtoC(const Matrix6d &N) {
  // Eigen::Matrix<double, 6, 9> E;
  // E <<
  //     1, 0, 0, 0, 0, 0, 0, 0, 0,
  //     0, 0, 0, 0, 1, 0, 0, 0, 0,
  //     0, 0, 0, 0, 0, 0, 0, 0, 1,
  //     0, 1, 0, 1, 0, 0, 0, 0, 0,
  //     0, 0, 1, 0, 0, 0, 1, 0, 0,
  //     0, 0, 0, 0, 0, 1, 0, 1, 0;  
  // const Matrix9d &&extN = E.transpose()*N*E;
  
  // Vector81d C;
  // for (int i = 0; i < 3; ++i) {
  //   for (int j = 0; j < 3; ++j) {
  //     for (int k = 0; k < 3; ++k) {
  //       for (int l = 0; l < 3; ++l) {
  //         C[rank4Index(i, j, k, l)] = extN(3*i+j, 3*k+l);
  //       }
  //     }
  //   }
  // }
  Vector81d C;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          C[rank4Index(i, j, k, l)] = N(voigt_map(i, j), voigt_map(k, l));
        }
      }
    }
  }  
  return C;
}

inline void calc_lame_const(const double Ym, const double Pr,
                            double &mu, double &lambda) {
  mu = Ym/(2*(1+Pr));
  lambda = Ym*Pr/((1+Pr)*(1-2*Pr));
}

inline void vox_SF(double *val, const double *epsilon) {
  const double 
      tt1 = 1-epsilon[0],
      tt2 = 1-epsilon[1],
      tt3 = 1-epsilon[2],
      tt4 = epsilon[0]+1,
      tt5 = epsilon[1]+1,
      tt6 = epsilon[2]+1;
  
  val[0] = (tt1*tt2*tt3)/8.0;
  val[1] = (tt4*tt2*tt3)/8.0;
  val[2] = (tt1*tt5*tt3)/8.0;
  val[3] = (tt4*tt5*tt3)/8.0;
  val[4] = (tt1*tt2*tt6)/8.0;
  val[5] = (tt4*tt2*tt6)/8.0;
  val[6] = (tt1*tt5*tt6)/8.0;
  val[7] = (tt4*tt5*tt6)/8.0;
}

inline void vox_SF_jac(double *jac, const double *epsilon) {
  const double
      tt1 = 1-epsilon[1],
      tt2 = 1-epsilon[2],
      tt3 = epsilon[1]+1,
      tt4 = epsilon[2]+1,
      tt5 = 1-epsilon[0],
      tt6 = epsilon[0]+1;

  jac[0] = -(tt1*tt2)/8.0;
  jac[1] = (tt1*tt2)/8.0;
  jac[2] = -(tt3*tt2)/8.0;
  jac[3] = (tt3*tt2)/8.0;
  jac[4] = -(tt1*tt4)/8.0;
  jac[5] = (tt1*tt4)/8.0;
  jac[6] = -(tt3*tt4)/8.0;
  jac[7] = (tt3*tt4)/8.0;
  jac[8] = -(tt5*tt2)/8.0;
  jac[9] = -(tt6*tt2)/8.0;
  jac[10] = (tt5*tt2)/8.0;
  jac[11] = (tt6*tt2)/8.0;
  jac[12] = -(tt5*tt4)/8.0;
  jac[13] = -(tt6*tt4)/8.0;
  jac[14] = (tt5*tt4)/8.0;
  jac[15] = (tt6*tt4)/8.0;
  jac[16] = -(tt5*tt1)/8.0;
  jac[17] = -(tt6*tt1)/8.0;
  jac[18] = -(tt5*tt3)/8.0;
  jac[19] = -(tt6*tt3)/8.0;
  jac[20] = (tt5*tt1)/8.0;
  jac[21] = (tt6*tt1)/8.0;
  jac[22] = (tt5*tt3)/8.0;
  jac[23] = (tt6*tt3)/8.0;
}

extern "C" {
  int dlaev2_(double *a, double *b, double *c, 
              double *rt1, double *rt2, double *cs1, double *sn1);

  int dgesvd_(char *jobu, char *jobvt, int *m, int *n, 
              double *a, int *lda, double *s, double *u, int *ldu, double *vt, 
              int *ldvt, double *work, int *lwork, int *info);  
}

inline int svd2d(Eigen::Matrix2d &A, Eigen::Matrix2d &U, Eigen::Vector2d &diagS, Eigen::Matrix2d &VT) {
  int m = 2, n = 2, info = -1, min_mn = 2;
  int lwork = std::max(3*min_mn+std::max(m,n), 5*min_mn);
  Eigen::VectorXd w(lwork);
  static char sS[] = "S";
  dgesvd_(sS, sS, &m, &n, A.data(), &m, diagS.data(), U.data(), &m,
          VT.data(), &min_mn, w.data(), &lwork, &info);
  return info;
}

inline void eig2d(Eigen::Matrix2d &A, Eigen::Matrix2d &V, Eigen::Vector2d &S) {
  double CS, SN;
  dlaev2_(&A(0, 0), &A(0, 1), &A(1, 1), &S[0], &S[1], &CS, &SN);  
  V(0, 0) = V(1, 1) = CS;
  V(0, 1) = -SN;
  V(1, 0) = SN;
}

inline Eigen::Matrix2d polar2d_a(const Eigen::Matrix2d &A) {
  Eigen::Matrix2d ATA = A.transpose()*A;
  Eigen::Vector2d S;
  double CS, SN;
  dlaev2_(&ATA(0, 0), &ATA(0, 1), &ATA(1, 1), &S[0], &S[1], &CS, &SN);

  Eigen::Matrix2d V;
  {
    V(0, 0) = V(1, 1) = CS;
    V(0, 1) = -SN;
    V(1, 0) = SN;
  }
  Eigen::Matrix2d US = A*V;
  US.col(0) /= sqrt(S[0]);
  US.col(1) /= sqrt(S[1]);

  if ( US.determinant()*V.determinant() < 0 ) { // reflection
    US.col(1) *= -1;
  }
  return US*V.transpose();
}

inline Eigen::Matrix2d polar2d_b(const Eigen::Matrix2d &M) {  
  Eigen::Matrix2d A = M, U, VT; Eigen::Vector2d diagS;
  int m = 2, n = 2, info = -1, min_mn = 2;
  int lwork = std::max(3*min_mn+std::max(m,n), 5*min_mn);
  Eigen::VectorXd w(lwork);
  static char sS[] = "S";
  dgesvd_(sS, sS, &m, &n, A.data(), &m, diagS.data(), U.data(), &m,
          VT.data(), &min_mn, w.data(), &lwork, &info);

  if ( U.determinant()*VT.determinant() < 0 ) {
    U.col(1) *= -1;
  }
  return U*VT;
}

inline Eigen::Matrix3d polar3d_a(const Eigen::Matrix3d &A, const int maxIter=5) {
  Eigen::Quaterniond q(A);
  q.normalize();
  for (unsigned int iter = 0; iter < maxIter; iter++) {
    Eigen::Matrix3d R = q.matrix();
    Eigen::Vector3d omega = (R.col(0).cross(A.col(0)) + R.col
                      (1).cross(A.col(1)) + R.col(2).cross(A.col(2))
                      ) * (1.0 / fabs(R.col(0).dot(A.col(0)) + R.col
                                      (1).dot(A.col(1)) + R.col(2).dot(A.col(2))) +
                           1.0e-9);
    double w = omega.norm();
    if (w < 1.0e-9)
      break;
    q = Eigen::Quaterniond(Eigen::AngleAxisd(w, (1.0 / w) * omega))*q;
    q.normalize();
  }
  return q.toRotationMatrix();
}

inline Eigen::Matrix3d polar3d_b(const Eigen::Matrix3d &M) {
  Eigen::Matrix3d A = M, U, VT; Eigen::Vector3d diagS;
  int m = 3, n = 3, info = -1, min_mn = 3;
  int lwork = std::max(3*min_mn+std::max(m,n), 5*min_mn);
  Eigen::VectorXd w(lwork);
  static char sS[] = "S";
  dgesvd_(sS, sS, &m, &n, A.data(), &m, diagS.data(), U.data(), &m,
          VT.data(), &min_mn, w.data(), &lwork, &info);

  if ( U.determinant()*VT.determinant() < 0 ) {
    U.col(1) *= -1;
  }
  return U*VT;
}

template <class Vec2> 
typename Vec2::Scalar quad_area_2d(const Vec2 &a,
                                   const Vec2 &b,
                                   const Vec2 &c,
                                   const Vec2 &d) {
  typename Vec2::Scalar s = 0;
  s += a.x()*b.y()-a.y()*b.x();
  s += b.x()*c.y()-b.y()*c.x();
  s += c.x()*d.y()-c.y()*d.x();
  s += d.x()*a.y()-d.y()*a.x();
  return 0.5*s;
}

template <class Vec3>
typename Vec3::Scalar tri_sgn_vol(const Vec3 &p1,
                                  const Vec3 &p2,
                                  const Vec3 &p3) {
  auto v321 = p3.x()*p2.y()*p1.z();
  auto v231 = p2.x()*p3.y()*p1.z();
  auto v312 = p3.x()*p1.y()*p2.z();
  auto v132 = p1.x()*p3.y()*p2.z();
  auto v213 = p2.x()*p1.y()*p3.z();
  auto v123 = p1.x()*p2.y()*p3.z();
  return (1.0/6)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

template <class Vec3>
typename Vec3::Scalar quad_sgn_vol(const Vec3 &p1,
                                   const Vec3 &p2,
                                   const Vec3 &p3,
                                   const Vec3 &p4) {
  return tri_sgn_vol<Vec3>(p1, p2, p3)+tri_sgn_vol<Vec3>(p3, p4, p1);
}

template <class Mat, class Vec>
int lapacksvd(Mat &A, Mat &U, Mat &VT, Vec &diagS) {
  int m = A.rows(), n = A.cols(), info = -1, min_mn = std::min(m, n);
  int lwork = std::max(3*min_mn+std::max(m,n), 5*min_mn);
  Eigen::VectorXd w(lwork);
  static char sS[] = "S";
  dgesvd_(sS, sS, &m, &n, A.data(), &m,
          diagS.data(), U.data(), &m,
          VT.data(), &min_mn, w.data(), &lwork, &info);
  return info;
}

}
#endif
