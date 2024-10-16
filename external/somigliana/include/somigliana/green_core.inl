
#include <Eigen/Dense>

typedef float scalar_t;
typedef int   index_t;

typedef Eigen::Matrix<scalar_t, 3, 1> Vec3;
typedef Eigen::Matrix<scalar_t, 3, 3> Mat3;

#define CUDA_PI 3.141592653589793f

inline 
#ifdef SOMIG_WITH_CUDA
__host__ __device__ 
#endif
int sgn(const scalar_t val) {
  return ((0.0 < val)-(val < 0.0));
}

inline 
#ifdef SOMIG_WITH_CUDA
__host__ __device__ 
#endif
scalar_t double_trig_area(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
  return ((b-a).cross(c-a)).norm();
}

 
#ifdef SOMIG_WITH_CUDA
__host__ __device__ 
#endif
void mvc_kernel(scalar_t *d_PHI,
                           const scalar_t *d_V,
                           const index_t  *d_cageF,
                           const scalar_t *d_cageV,
                           const index_t nv,
                           const index_t ncf,
                           const index_t ncv,
                           const index_t index) {
  // unsigned int index = blockIdx.x*blockDim.x + threadIdx.x;

  if ( index < nv ) {
    Vec3 x(d_V[3*index+0], d_V[3*index+1], d_V[3*index+2]);

    // for each face
    for (index_t f = 0; f < ncf; ++f) {
      index_t vx = d_cageF[3*f+0], vy = d_cageF[3*f+1], vz = d_cageF[3*f+2];

      Mat3 u;
      {
        u.col(0) = Vec3(d_cageV[3*vx+0], d_cageV[3*vx+1], d_cageV[3*vx+2])-x;
        u.col(1) = Vec3(d_cageV[3*vy+0], d_cageV[3*vy+1], d_cageV[3*vy+2])-x;
        u.col(2) = Vec3(d_cageV[3*vz+0], d_cageV[3*vz+1], d_cageV[3*vz+2])-x;
      }
      Vec3 d(u.col(0).norm(), u.col(1).norm(), u.col(2).norm());
      u.colwise().normalize();
      scalar_t sgn_u = sgn(u.determinant());
      
      Vec3 l, theta, c, s;
      {
        l[0] = (u.col(1)-u.col(2)).norm();
        l[1] = (u.col(2)-u.col(0)).norm();
        l[2] = (u.col(0)-u.col(1)).norm();

        theta[0] = 2*asin(l[0]/2);
        theta[1] = 2*asin(l[1]/2);
        theta[2] = 2*asin(l[2]/2);
      }

      const scalar_t h = theta.sum()/2;      

      const scalar_t TOL = 1e-8;
      if ( fabs(CUDA_PI-h) < TOL ) {
        d_PHI[vx+ncv*index] += sin(theta[0])*d[1]*d[2];
        d_PHI[vy+ncv*index] += sin(theta[1])*d[2]*d[0];
        d_PHI[vz+ncv*index] += sin(theta[2])*d[0]*d[1];
      } else {        
        c[0] = 2*sin(h)*sin(h-theta[0])
            /(sin(theta[1])*sin(theta[2]))-1;
        c[1] = 2*sin(h)*sin(h-theta[1])
            /(sin(theta[2])*sin(theta[0]))-1;
        c[2] = 2*sin(h)*sin(h-theta[2])
            /(sin(theta[0])*sin(theta[1]))-1; 

        s[0] = sgn_u*sqrt(1-c[0]*c[0]);
        s[1] = sgn_u*sqrt(1-c[1]*c[1]);
        s[2] = sgn_u*sqrt(1-c[2]*c[2]);

        if ( fabs(s[0]) < TOL || fabs(s[1]) < TOL || fabs(s[2]) < TOL ) {
          continue;
        }
        
        // assign
        d_PHI[vx+ncv*index] += (theta[0]-c[2]*theta[1]-c[1]*theta[2])/(d[0]*sin(theta[2])*s[1]); 
        d_PHI[vy+ncv*index] += (theta[1]-c[0]*theta[2]-c[2]*theta[0])/(d[1]*sin(theta[0])*s[2]); 
        d_PHI[vz+ncv*index] += (theta[2]-c[1]*theta[0]-c[0]*theta[1])/(d[2]*sin(theta[1])*s[0]); 
      }      
    } // end for f    
  } // end if index...
}

 
#ifdef SOMIG_WITH_CUDA
__host__ __device__ 
#endif
void green_kernel(scalar_t *d_phix,
                             scalar_t *d_phiy,
                             scalar_t *d_phiz,
                             scalar_t *d_psi,
                             const scalar_t *d_V,
                             const index_t  *d_cageF,
                             const scalar_t *d_cageV,
                             const scalar_t *d_cageN,
                             const index_t nv,
                             const index_t ncf,
                             const index_t ncv,
                             const scalar_t *d_qp,
                             const scalar_t *d_qw,
                             const index_t nq,
                             //const unsigned int thread_index,
                             const unsigned int index,
                             const unsigned int f) {
  // unsigned int thread_index = blockIdx.x*blockDim.x + threadIdx.x;
  // unsigned int index = thread_index/ncf, f = thread_index%ncf;

  if ( f < ncf && index < nv ) {
    const index_t ix = d_cageF[3*f+0], iy = d_cageF[3*f+1], iz = d_cageF[3*f+2];
    Vec3 x (d_V[3*index+0],  d_V[3*index+1],  d_V[3*index+2]);
    Vec3 v0(d_cageV[3*ix+0], d_cageV[3*ix+1], d_cageV[3*ix+2]);
    Vec3 v1(d_cageV[3*iy+0], d_cageV[3*iy+1], d_cageV[3*iy+2]);
    Vec3 v2(d_cageV[3*iz+0], d_cageV[3*iz+1], d_cageV[3*iz+2]);
    Vec3 n (d_cageN[3*f+0],  d_cageN[3*f+1],  d_cageN[3*f+2]);   
    const scalar_t absA = fabs(double_trig_area(v0, v1, v2));

    Vec3 v, w, d;
    {
      v = v1-v0;
      w = v2-v0;
      d = v0-x;
    }

    const scalar_t C0 = 1/(4*CUDA_PI);
    
    scalar_t psi, phi0, phi1, phi2;
    psi = phi0 = phi1 = phi2 = 0;
    for (index_t i = 0; i < nq; ++i) {
      const scalar_t a = d_qp[2*i+0], b = d_qp[2*i+1], weight = d_qw[i];
      const Vec3 r = d+a*v+b*w;
      const scalar_t i_rn = 1.0/r.norm(), i_rn3 = i_rn*i_rn*i_rn;

      scalar_t G = C0*i_rn;
      scalar_t dG = C0*i_rn3*r.dot(n);
      
      psi  += weight*G;
      phi0 += weight*(1-a-b)*dG;
      phi1 += weight*a*dG;
      phi2 += weight*b*dG;
    }

    // assign
    d_psi [f+ncf*index] = absA*psi;
    d_phix[f+ncf*index] = absA*phi0;
    d_phiy[f+ncf*index] = absA*phi1;
    d_phiz[f+ncf*index] = absA*phi2;
  } // end if index
}

 
#ifdef SOMIG_WITH_CUDA
__host__ __device__ 
#endif
 void green_kernel_post(scalar_t *d_phi,
                                  const scalar_t *d_phix,
                                  const scalar_t *d_phiy,
                                  const scalar_t *d_phiz,
                                  const index_t *d_cageF,
                                  const index_t nv,
                                  const index_t ncf,
                                  const index_t ncv,
                           const unsigned int index) {
  // unsigned int index = blockIdx.x*blockDim.x + threadIdx.x;

  if ( index < nv ) {
    for (index_t f = 0; f < ncf; ++f) { // for each face
      d_phi[ncv*index+d_cageF[3*f+0]] += d_phix[ncf*index+f];
      d_phi[ncv*index+d_cageF[3*f+1]] += d_phiy[ncf*index+f];
      d_phi[ncv*index+d_cageF[3*f+2]] += d_phiz[ncf*index+f];      
    } // endfor
  } // endif 
}
 
#ifdef SOMIG_WITH_CUDA
__host__ __device__ 
#endif
 void somig_kernel(const scalar_t nu,
                             scalar_t *d_PHIx,
                             scalar_t *d_PHIy,
                             scalar_t *d_PHIz,
                             scalar_t *d_PSI ,
                             const scalar_t *d_V,
                             const index_t  *d_cageF,
                             const scalar_t *d_cageV,
                             const scalar_t *d_cageN,
                             const index_t nv,
                             const index_t ncf,
                             const index_t ncv,
                             const scalar_t *d_qp,
                             const scalar_t *d_qw,
                             const index_t nq,
                             //const unsigned int thread_index,
                             const unsigned int index,
                             const unsigned int f)  {
  // unsigned int thread_index = blockIdx.x*blockDim.x + threadIdx.x;
  // unsigned int index = thread_index/ncf, f = thread_index%ncf;

  if ( f < ncf && index < nv ) {
    const scalar_t mu = 1e0;
    const scalar_t C1 = 1.0/(16*CUDA_PI*mu*(1-nu));
    const scalar_t C2 = -1.0/(8*CUDA_PI*(1-nu));
    const Mat3 Id = Mat3::Identity();
    
    const index_t ix = d_cageF[3*f+0], iy = d_cageF[3*f+1], iz = d_cageF[3*f+2];
    Vec3 x (d_V[3*index+0],  d_V[3*index+1],  d_V[3*index+2]);
    Vec3 v0(d_cageV[3*ix+0], d_cageV[3*ix+1], d_cageV[3*ix+2]);
    Vec3 v1(d_cageV[3*iy+0], d_cageV[3*iy+1], d_cageV[3*iy+2]);
    Vec3 v2(d_cageV[3*iz+0], d_cageV[3*iz+1], d_cageV[3*iz+2]);
    Vec3 n (d_cageN[3*f+0],  d_cageN[3*f+1],  d_cageN[3*f+2]);

    const scalar_t absA = fabs(double_trig_area(v0, v1, v2));

    Vec3 v, w, d;
    {
      v = v1-v0;
      w = v2-v0;
      d = v0-x;
    }

    const scalar_t c_12nu = 1-2*nu, c_34nu = 3-4*nu;
    
    Mat3 psi, phi0, phi1, phi2;
    psi = phi0 = phi1 = phi2 = Mat3::Zero();
    Mat3 rrT, tmp;
    for (index_t i = 0; i < nq; ++i) {
      const scalar_t a = d_qp[2*i+0], b = d_qp[2*i+1], weight = d_qw[i];
      const Vec3 r = d+a*v+b*w;
      const scalar_t i_rn = 1.0/r.norm(), i_rn3 = i_rn*i_rn*i_rn,
          i_rn5 = i_rn3*i_rn*i_rn;

      rrT = r*r.transpose();
      tmp = (c_12nu*r.dot(n)*i_rn3*Id+
             3*r.dot(n)*i_rn5*rrT-
             c_12nu*i_rn3*(r*n.transpose()-n*r.transpose()));            
      
      psi  += weight*(c_34nu*i_rn*Id+rrT*i_rn3);
      phi0 += weight*(1-a-b)*tmp;
      phi1 += weight*a*tmp;
      phi2 += weight*b*tmp;
    }

    psi  *=  C1*absA;
    phi0 *= -C2*absA;
    phi1 *= -C2*absA;
    phi2 *= -C2*absA;

    scalar_t* ptr[4] = {d_PSI, d_PHIx, d_PHIy, d_PHIz};
    Mat3*     cof[4] = {&psi,  &phi0,  &phi1,  &phi2};
    for (index_t i = 0; i < 4; ++i) {
      *(ptr[i]+3*nv*(3*f+0)+3*index+0) += (*cof[i])(0, 0);
      *(ptr[i]+3*nv*(3*f+0)+3*index+1) += (*cof[i])(1, 0);
      *(ptr[i]+3*nv*(3*f+0)+3*index+2) += (*cof[i])(2, 0);

      *(ptr[i]+3*nv*(3*f+1)+3*index+0) += (*cof[i])(0, 1); 
      *(ptr[i]+3*nv*(3*f+1)+3*index+1) += (*cof[i])(1, 1); 
      *(ptr[i]+3*nv*(3*f+1)+3*index+2) += (*cof[i])(2, 1); 

      *(ptr[i]+3*nv*(3*f+2)+3*index+0) += (*cof[i])(0, 2);
      *(ptr[i]+3*nv*(3*f+2)+3*index+1) += (*cof[i])(1, 2);
      *(ptr[i]+3*nv*(3*f+2)+3*index+2) += (*cof[i])(2, 2);
    }
  } // end if
}

 
#ifdef SOMIG_WITH_CUDA
__host__ __device__ 
#endif
 void somig_kernel_post(scalar_t *d_PHI,
                                  const scalar_t *d_PHIx,
                                  const scalar_t *d_PHIy,
                                  const scalar_t *d_PHIz,
                                  const index_t *d_cageF,
                                  const index_t nv,
                                  const index_t ncf,
                                  const unsigned int index) {
  // unsigned int index = blockIdx.x*blockDim.x + threadIdx.x;

  if ( index < nv ) {
    const scalar_t* ptr[3] = {d_PHIx, d_PHIy, d_PHIz};

    for (index_t f = 0; f < ncf; ++f) { // for each face
      for (index_t i = 0; i < 3; ++i) {
        const index_t v = d_cageF[3*f+i];
        d_PHI[3*nv*(3*v+0)+3*index+0] += ptr[i][3*nv*(3*f+0)+3*index+0];
        d_PHI[3*nv*(3*v+0)+3*index+1] += ptr[i][3*nv*(3*f+0)+3*index+1];
        d_PHI[3*nv*(3*v+0)+3*index+2] += ptr[i][3*nv*(3*f+0)+3*index+2];

        d_PHI[3*nv*(3*v+1)+3*index+0] += ptr[i][3*nv*(3*f+1)+3*index+0];
        d_PHI[3*nv*(3*v+1)+3*index+1] += ptr[i][3*nv*(3*f+1)+3*index+1];
        d_PHI[3*nv*(3*v+1)+3*index+2] += ptr[i][3*nv*(3*f+1)+3*index+2];

        d_PHI[3*nv*(3*v+2)+3*index+0] += ptr[i][3*nv*(3*f+2)+3*index+0];
        d_PHI[3*nv*(3*v+2)+3*index+1] += ptr[i][3*nv*(3*f+2)+3*index+1];
        d_PHI[3*nv*(3*v+2)+3*index+2] += ptr[i][3*nv*(3*f+2)+3*index+2];
      }
    } // endfor
  } // endif 
}
