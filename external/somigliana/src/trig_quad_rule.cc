#include <somigliana/trig_quad_rule.h>

#include <iostream>
#include <cmath>
#include <vector>
#include <somigliana/quadrule.hpp>

using namespace Eigen;
using namespace std;

namespace green {

// 3----2
// |    |
// 0----1
static inline Eigen::Vector4d shape_func(const double *s) {
  return Vector4d((1-s[0])*(1-s[1])/4.0,
                  (1+s[0])*(1-s[1])/4.0,
                  (1+s[0])*(1+s[1])/4.0,
                  (1-s[0])*(1+s[1])/4.0);
}

typedef Eigen::Matrix<double, 4, 2> Matrix42;

static inline Matrix42 shape_func_gra(const double *s) {
  Matrix42 res;
  res(0, 0) = -(1-s[1])/4.0; res(0, 1) = -(1-s[0])/4.0;
  res(1, 0) =  (1-s[1])/4.0; res(1, 1) = -(1+s[0])/4.0;
  res(2, 0) =  (1+s[1])/4.0; res(2, 1) =  (1+s[0])/4.0;
  res(3, 0) = -(1+s[1])/4.0; res(3, 1) =  (1-s[0])/4.0;  
  return res;
}

void sym_trig_quad_rule(const size_t number,
                        Matrix2Xd &q,
                        VectorXd  &w) {
  // compute 1d Gauss quadrature [-1,1]
  const size_t n = sqrt(number/3);
  // cout << "#quadrature n=" << n << endl;
  
  VectorXd q1(n), w1(n);
  cgqf(n, 1, 0, 0, -1, 1, -1, q1.data(), w1.data());

  // compute 2d Gauss quadrature [-1,1]x[-1,1]
  Matrix2Xd q2(2, n*n); VectorXd w2(n*n);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      size_t index = i*n+j;
      q2(0, index) = q1[i];
      q2(1, index) = q1[j];
      w2[index]    = w1[i]*w1[j];
    }
  }

  // trun triangle into 3 quads, 7 nodes
  Matrix2Xd nods(2, 7);
  {
    nods(0, 0) = 0;     nods(1, 0) = 0;
    nods(0, 1) = 1;     nods(1, 1) = 0;
    nods(0, 2) = 0;     nods(1, 2) = 1;
    nods(0, 3) = 0.5;   nods(1, 3) = 0;
    nods(0, 4) = 0.5;   nods(1, 4) = 0.5;
    nods(0, 5) = 0;     nods(1, 5) = 0.5;
    nods(0, 6) = 1.0/3; nods(1, 6) = 1.0/3;
  }
  MatrixXi quad(4, 3);
  {
    quad(0, 0) = 0; quad(0, 1) = 3; quad(0, 2) = 5;
    quad(1, 0) = 3; quad(1, 1) = 1; quad(1, 2) = 6;
    quad(2, 0) = 6; quad(2, 1) = 4; quad(2, 2) = 4;
    quad(3, 0) = 5; quad(3, 1) = 6; quad(3, 2) = 2;
  }

  size_t count = 0;  
  q.resize(2, 3*n*n);
  w.resize(3*n*n);
  for (size_t i = 0; i < n*n; ++i) { //  for each 2d quadrature
    for (size_t f = 0; f < 3; ++f) {
      const MatrixXd v = nods(Eigen::all, quad.col(f).array());

      // quadrature point
      q.col(count) = v*shape_func(&q2(0, i));
      
      // quadrature weight
      const Matrix2d Q = v*shape_func_gra(&q2(0, i));
      w(count) = w2(i)*fabs(Q.determinant());

      ++count;
    }
  }
  // cout << "#quadrature=" << count << endl;
}

}
