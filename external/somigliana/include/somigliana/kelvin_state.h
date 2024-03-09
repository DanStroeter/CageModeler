#ifndef KELVIN_STATE_H
#define KELVIN_STATE_H

#include <Eigen/Eigen>

namespace green {

typedef Eigen::Matrix2d Mat2;
typedef Eigen::Matrix3d Mat3;  

typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;

typedef Eigen::Matrix<double, 4, 2> Mat4x2;
typedef Eigen::Matrix<double, 9, 3> Mat9x3;


template <class Vec, class Mat>
void outer_product(const Vec &a,
                   const double s,
                   const Mat &B,
                   std::vector<Mat> &C) {
  for (size_t i = 0; i < C.size(); ++i) {
    C[i] += s*a(i)*B;
  }
}

template <class Vec, class Mat>
void vec_distr_col(const Vec &a,
                   const double s,
                   std::vector<Mat> &C) {
  for (size_t i = 0; i < C.size(); ++i) {
    C[i].col(i) += s*a;
  }
}

template <class Vec, class Mat>
void vec_distr_row(const Vec &a,
                   const double s,
                   std::vector<Mat> &C) {
  for (size_t i = 0; i < C.size(); ++i) {
    C[i].row(i) += s*a;
  }
}

template <class Mat>
std::vector<Mat> view_k2j(const std::vector<Mat> &t) {
  const int d = t.size();
  std::vector<Mat> ret(t.size(), Mat::Zero());
  for (int i = 0; i < d; ++i) {
    for (int j = 0; j < d; ++j) {
      for (int k = 0; k < d; ++k) {
        ret[j](i, k) = t[k](i, j);
      }
    }
  }
  return ret;
}

struct kelvin_displ
{
  double mu_, nu_;
  const Mat2 Id2_ = Mat2::Identity();
  const Mat3 Id3_ = Mat3::Identity();  

  kelvin_displ(const double mu, const double nu) : mu_(mu), nu_(nu) {}

  // 2D case
  Mat2 K(const Vec2 &r) const {
    const double rn = r.norm(), rn2 = rn*rn;
    return 1.0/(8*M_PI*mu_*(1-nu_))*(
        -(3-4*nu_)*log(rn)*Id2_
        +1.0/rn2*r*r.transpose());
  }
  Mat2 K_R(const Vec2 &r) const {
    const double rn = r.norm(), rn2 = rn*rn;
    return 1.0/(8*M_PI*mu_*(1-nu_))*(
        -(3-4*nu_)*log(rn)*Id2_);    
  }
  std::vector<Mat2> dKdy(const Vec2 &r) const {
    std::vector<Mat2> rtn(2, Mat2::Zero());
    const double rn = r.norm(), rn2 = rn*rn, rn4 = rn2*rn2;
    const double C1 = 1.0/(8*M_PI*mu_*(1-nu_)), C2 = -C1*(3-4*nu_);
    outer_product<Vec2, Mat2>(r/rn2, C2, Id2_, rtn);
    outer_product<Vec2, Mat2>(-2*r/rn4, C1, r*r.transpose(), rtn);
    vec_distr_col<Vec2, Mat2>(r, C1/rn2, rtn);
    vec_distr_row<Vec2, Mat2>(r, C1/rn2, rtn);
    return rtn;
  }
  void dKdy(const Vec2 &r, const double s, std::vector<Mat2> &rtn) const {
    const double rn = r.norm(), rn2 = rn*rn, rn4 = rn2*rn2;
    const double C1 = 1.0/(8*M_PI*mu_*(1-nu_)), C2 = -C1*(3-4*nu_);
    outer_product<Vec2, Mat2>(r/rn2, s*C2, Id2_, rtn);
    outer_product<Vec2, Mat2>(-2*r/rn4, s*C1, r*r.transpose(), rtn);
    vec_distr_col<Vec2, Mat2>(r, s*C1/rn2, rtn);
    vec_distr_row<Vec2, Mat2>(r, s*C1/rn2, rtn);
  }
  void dKdx(const Vec2 &r, const double s, std::vector<Mat2> &rtn) const {
    const double rn = r.norm(), rn2 = rn*rn, rn4 = rn2*rn2;
    const double C1 = 1.0/(8*M_PI*mu_*(1-nu_)), C2 = -C1*(3-4*nu_);
    outer_product<Vec2, Mat2>(r/rn2, -s*C2, Id2_, rtn);
    outer_product<Vec2, Mat2>(-2*r/rn4, -s*C1, r*r.transpose(), rtn);
    vec_distr_col<Vec2, Mat2>(r, -s*C1/rn2, rtn);
    vec_distr_row<Vec2, Mat2>(r, -s*C1/rn2, rtn);    
  }

  // 3D case
  Mat3 K(const Vec3 &r) const {
    const double rn = r.norm(), rn3 = rn*rn*rn;
    return 1.0/(16*M_PI*mu_*(1-nu_))*(
        (3-4*nu_)/rn*Id3_+
        1.0/rn3*r*r.transpose());
  }
  std::vector<Mat3> dKdy(const Vec3 &r) const {
    std::vector<Mat3> rtn(3, Mat3::Zero());
    const double rn = r.norm(), rn2 = rn*rn, rn3 = rn2*rn, rn5 = rn3*rn2;
    const double C1 = 1.0/(16*M_PI*mu_*(1-nu_)), C2 = C1*(3-4*nu_);
    outer_product<Vec3, Mat3>(-1*r/rn3, C2, Id3_, rtn);
    outer_product<Vec3, Mat3>(-3*r/rn5, C1, r*r.transpose(), rtn);
    vec_distr_col<Vec3, Mat3>(r, C1/rn3, rtn);
    vec_distr_row<Vec3, Mat3>(r, C1/rn3, rtn);
    return rtn;
  }
};

struct kelvin_traction
{
  double mu_, nu_;
  const Mat2 Id2_ = Mat2::Identity();
  const Mat3 Id3_ = Mat3::Identity();

  kelvin_traction(const double mu, const double nu) : mu_(mu), nu_(nu) {}

  // 2D
  Mat2 T(const Vec2 &r, const Vec2 &n) const {
    const double rn = r.norm(), rn2 = rn*rn, rn3 = rn2*rn;
    const double C0 = -1/(4*M_PI*(1-nu_));    
    return C0*(r.dot(n)/rn*((1-2*nu_)/rn*Id2_+2/rn3*r*r.transpose())
               -(1-2*nu_)/rn2*(r*n.transpose()-n*r.transpose()));
  }
  Mat2 T_R(const Vec2 &r, const Vec2 &n) const {
    const double rn = r.norm(), rn2 = rn*rn, rn3 = rn2*rn;
    const double C0 = -1/(4*M_PI*(1-nu_));    
    return C0*(r.dot(n)/rn*((1-2*nu_)/rn*Id2_)
               -(1-2*nu_)/rn2*(r*n.transpose()-n*r.transpose()));
  }  
  std::vector<Mat2> dTdy(const Vec2 &r, const Vec2 &n) const {
    std::vector<Mat2> rtn(2, Mat2::Zero());    
    const double rn = r.norm(), rn2 = rn*rn, rn3 = rn2*rn, rn4 = rn2*rn2, rn5 = rn3*rn2;
    const double C0 = -1/(4*M_PI*(1-nu_)), C1 = 1-2*nu_;
    const Mat2 rrT = r*r.transpose(), rnnr = r*n.transpose()-n*r.transpose();
    const double drdn = r.dot(n)/rn;
    
    outer_product<Vec2, Mat2>(-r.dot(n)*r/rn3+n/rn, C0, C1/rn*Id2_+2/rn3*rrT, rtn);
    outer_product<Vec2, Mat2>(-r/rn3, C1*C0*drdn, Id2_, rtn);
    outer_product<Vec2, Mat2>(-3*r/rn5, 2*C0*drdn, rrT, rtn);
    vec_distr_col<Vec2, Mat2>(r, 2*C0*drdn/rn3, rtn);
    vec_distr_row<Vec2, Mat2>(r, 2*C0*drdn/rn3, rtn);
    outer_product<Vec2, Mat2>(-2*r/rn4, -C0*C1, rnnr, rtn);
    vec_distr_row<Vec2, Mat2>(n, -C0*C1/rn2, rtn);    
    vec_distr_col<Vec2, Mat2>(n, C0*C1/rn2, rtn);
    return rtn;
  }
  void dTdy(const Vec2 &r, const Vec2 &n, const double s, std::vector<Mat2> &rtn) const {
    const double rn = r.norm(), rn2 = rn*rn, rn3 = rn2*rn, rn4 = rn2*rn2, rn5 = rn3*rn2;
    const double C0 = -1/(4*M_PI*(1-nu_)), C1 = 1-2*nu_;
    const Mat2 rrT = r*r.transpose(), rnnr = r*n.transpose()-n*r.transpose();
    const double drdn = r.dot(n)/rn;
    
    outer_product<Vec2, Mat2>(-r.dot(n)*r/rn3+n/rn, s*C0, C1/rn*Id2_+2/rn3*rrT, rtn);
    outer_product<Vec2, Mat2>(-r/rn3, s*C1*C0*drdn, Id2_, rtn);
    outer_product<Vec2, Mat2>(-3*r/rn5, s*2*C0*drdn, rrT, rtn);
    vec_distr_col<Vec2, Mat2>(r, s*2*C0*drdn/rn3, rtn);
    vec_distr_row<Vec2, Mat2>(r, s*2*C0*drdn/rn3, rtn);
    outer_product<Vec2, Mat2>(-2*r/rn4, -s*C0*C1, rnnr, rtn);
    vec_distr_row<Vec2, Mat2>(n, -s*C0*C1/rn2, rtn);    
    vec_distr_col<Vec2, Mat2>(n, s*C0*C1/rn2, rtn);
  }
  void dTdx(const Vec2 &r, const Vec2 &n, const double s, std::vector<Mat2> &rtn) const {
    const double rn = r.norm(), rn2 = rn*rn, rn3 = rn2*rn, rn4 = rn2*rn2, rn5 = rn3*rn2;
    const double C0 = -1/(4*M_PI*(1-nu_)), C1 = 1-2*nu_;
    const Mat2 rrT = r*r.transpose(), rnnr = r*n.transpose()-n*r.transpose();
    const double drdn = r.dot(n)/rn;
    
    outer_product<Vec2, Mat2>(-r.dot(n)*r/rn3+n/rn, -s*C0, C1/rn*Id2_+2/rn3*rrT, rtn);
    outer_product<Vec2, Mat2>(-r/rn3, -s*C1*C0*drdn, Id2_, rtn);
    outer_product<Vec2, Mat2>(-3*r/rn5, -s*2*C0*drdn, rrT, rtn);
    vec_distr_col<Vec2, Mat2>(r, -s*2*C0*drdn/rn3, rtn);
    vec_distr_row<Vec2, Mat2>(r, -s*2*C0*drdn/rn3, rtn);
    outer_product<Vec2, Mat2>(-2*r/rn4, s*C0*C1, rnnr, rtn);
    vec_distr_row<Vec2, Mat2>(n, s*C0*C1/rn2, rtn);    
    vec_distr_col<Vec2, Mat2>(n, -s*C0*C1/rn2, rtn);    
  }

  // 3D
  Mat3 T(const Vec3 &r, const Vec3 &n) const {
    const double rn = r.norm(), rn2 = rn*rn, rn3 = rn2*rn, rn4 = rn2*rn2;
    const double C0 = -1/(8*M_PI*(1-nu_));
    return C0*(r.dot(n)/rn*((1-2*nu_)/rn2*Id3_
                            +3/rn4*r*r.transpose())
               -(1-2*nu_)/rn3*(r*n.transpose()-n*r.transpose()));
  }
  std::vector<Mat3> dTdy(const Vec3 &r, const Vec3 &n) const {
    std::vector<Mat3> rtn(3, Mat3::Zero());
    const double rn = r.norm(), rn2 = rn*rn, rn3 = rn2*rn, rn4 = rn2*rn2, rn5 = rn3*rn2, rn6 = rn3*rn3;
    const double C0 = -1/(8*M_PI*(1-nu_)), C1 = 1-2*nu_;
    const Mat3 rrT = r*r.transpose(), rnnr = r*n.transpose()-n*r.transpose();
    const double drdn = r.dot(n)/rn;

    outer_product<Vec3, Mat3>(-r.dot(n)*r/rn3+n/rn, C0, C1/rn2*Id3_+3/rn4*rrT, rtn);
    outer_product<Vec3, Mat3>(-2*r/rn4, C1*C0*drdn, Id3_, rtn);
    outer_product<Vec3, Mat3>(-4*r/rn6, 3*C0*drdn, rrT, rtn);
    vec_distr_col<Vec3, Mat3>(r, 3*C0*drdn/rn4, rtn);
    vec_distr_row<Vec3, Mat3>(r, 3*C0*drdn/rn4, rtn);
    outer_product<Vec3, Mat3>(-3*r/rn5, -C0*C1, rnnr, rtn);
    vec_distr_row<Vec3, Mat3>(n, -C0*C1/rn3, rtn);
    vec_distr_col<Vec3, Mat3>(n, C0*C1/rn3, rtn);
    return rtn;
  }
};

}

#endif
