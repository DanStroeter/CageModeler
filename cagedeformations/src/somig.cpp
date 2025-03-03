#include "cagedeformations/somig.h"

#include "cagedeformations/BHC.h"

using namespace std;
using namespace Eigen;

extern "C" {
  int dgesvd_(char *jobu, char *jobvt, int *m, int *n, 
              double *a, int *lda, double *s, double *u, int *ldu, double *vt, 
              int *ldvt, double *work, int *lwork, int *info);  
}

template <class Mat, class Vec>
static int lapacksvd(Mat &A, Mat &U, Mat &VT, Vec &diagS) {
  int m = A.rows(), n = A.cols(), info = -1, min_mn = std::min(m, n);
  int lwork = std::max(3*min_mn+std::max(m,n), 5*min_mn);
  Eigen::VectorXd w(lwork);
  static char sS[] = "S";
  dgesvd_(sS, sS, &m, &n, A.data(), &m,
          diagS.data(), U.data(), &m,
          VT.data(), &min_mn, w.data(), &lwork, &info);
  return info;
}

void somig_deformer_3::set_mesh(const face_t &face, const vert_t &vert)
{
  F_.resize(3, face.size());
  for (size_t i = 0; i < face.size(); ++i) {
    F_(0, i) = face[i][0];
    F_(1, i) = face[i][1];
    F_(2, i) = face[i][2];    
  }

  V_.resize(3, vert.size());
  for (size_t i = 0; i < vert.size(); ++i) {
    V_(0, i) = vert[i].x();
    V_(1, i) = vert[i].y();
    V_(2, i) = vert[i].z();    
  }
}

void somig_deformer_3::set_cage(const face_t &face, const vert_t &vert)
{
  cageF_.resize(3, face.size());
  for (size_t i = 0; i < face.size(); ++i) {
    cageF_(0, i) = face[i][0];
    cageF_(1, i) = face[i][1];
    cageF_(2, i) = face[i][2];    
  }

  cageV_.resize(3, vert.size());
  for (size_t i = 0; i < vert.size(); ++i) {
    cageV_(0, i) = vert[i].x();
    cageV_(1, i) = vert[i].y();
    cageV_(2, i) = vert[i].z();    
  }
}

void somig_deformer_3::precompute_somig_coords(const bool sanity_check)
{
  // record rest states
  V0_ = V_;
  cageV0_ = cageV_;
  calc_outward_normal();
  cageN0_ = cageN_;
  
  const size_t ncv = num_cage_vertices(), nv = num_mesh_vertices(), nf = num_cage_facets();
  cout << "ncv, nv, nf=" << ncv << " " << nv << " " << nf << endl;

  typedef double T;
  typedef Vector3d point_t;

  auto cross =
      [](const point_t &x, const point_t &y) -> point_t {
        return x.cross(y);
      };

  auto dot =
      [](const point_t &x, const point_t &y) -> T {
        return x.dot(y);
      };

  auto get_signed_solid_angle =
      [&](point_t const & a, point_t const & b, point_t const & c) -> double {
        T det = dot(a, cross(b,c));
        if( fabs(det) < 0.0000000001 ) 
          return 2.0 * M_PI;

        T al = a.norm(),   bl = b.norm(),   cl = c.norm();

        T div = al*bl*cl + dot(a,b)*cl + dot(a,c)*bl + dot(b,c)*al;
        T at = atan2( fabs(det) , div );
        if(at < 0) at += M_PI; // If det>0 && div<0 atan2 returns < 0, so add pi.
        T omega = 2.0 * at;

        if(det > 0.0) return omega;
        return -omega;
      };

  // harmonic part: URAGO's approach  
  matd_t psi_, phi_;
  psi_.setZero(nf, nv);
  phi_.setZero(ncv, nv);
  MatrixXd phi0 = MatrixXd::Zero(nf, nv), phi1 = phi0, phi2 = phi0; // face-based buffers
  #pragma omp parallel for
  for (int pid = 0; pid < V0_.cols(); ++pid) {
    const point_t eta = V0_.col(pid);
    for (size_t i = 0; i < cageF_.cols(); ++i) {
      point_t tri_vertices[3] = {cageV0_.col(cageF_(0, i)),
                                 cageV0_.col(cageF_(1, i)),
                                 cageV0_.col(cageF_(2, i))};

      point_t Nt = cross( tri_vertices[1]-tri_vertices[0], tri_vertices[2]-tri_vertices[0]);
      T NtNorm = Nt.norm();
      T At = NtNorm / 2.0;
      Nt /= NtNorm;

      point_t e[3];    T e_norm[3];   point_t e_normalized[3];    T R[3];    point_t d[3];    T d_norm[3];     T C[3];     point_t J[3];
      for( unsigned int v = 0 ; v < 3 ; ++v ) e[v] = tri_vertices[v] - eta;
      for( unsigned int v = 0 ; v < 3 ; ++v ) e_norm[v] =  e[v].norm();
      for( unsigned int v = 0 ; v < 3 ; ++v ) e_normalized[v] = e[v] / e_norm[v];

      T signed_solid_angle = get_signed_solid_angle (e_normalized[0], e_normalized[1], e_normalized[2]) / (4.f * M_PI);
      T signed_volume = dot(cross(e[0],e[1]) , e[2] ) / 6.0;

      for( unsigned int v = 0 ; v < 3 ; ++v ) R[v] = e_norm[(v+1)%3] + e_norm[(v+2)%3];
      for( unsigned int v = 0 ; v < 3 ; ++v ) d[v] = tri_vertices[(v+1)%3] - tri_vertices[(v+2)%3];
      for( unsigned int v = 0 ; v < 3 ; ++v ) d_norm[v] =  d[v].norm();
      for( unsigned int v = 0 ; v < 3 ; ++v ) C[v] = log( (R[v] + d_norm[v]) / (R[v] - d_norm[v]) ) / (4.0 * M_PI * d_norm[v]);

      point_t Pt( - signed_solid_angle * Nt );
      for( unsigned int v = 0 ; v < 3 ; ++v ) Pt += cross( Nt , C[v]*d[v] );
      for( unsigned int v = 0 ; v < 3 ; ++v ) J[v] = cross( e[(v+2)%3] , e[(v+1)%3] );

      psi_(i, pid) = - 3.0 * signed_solid_angle * signed_volume / At ;
      for( unsigned int v = 0 ; v < 3 ; ++v ) psi_(i, pid) -= C[v]* dot(J[v],Nt);
      for( unsigned int v = 0 ; v < 3 ; ++v ) {
        phi_(cageF_(v, i), pid) += dot(Pt , J[v]) / (2.0 * At);
        if ( v == 0 ) phi0(i, pid) = dot(Pt , J[v]) / (2.0 * At);
        if ( v == 1 ) phi1(i, pid) = dot(Pt , J[v]) / (2.0 * At);
        if ( v == 2 ) phi2(i, pid) = dot(Pt , J[v]) / (2.0 * At);
      }
    }
  }

  // SC via Eq.(49), forget about the quadratures:)
  PSI_ = MatrixXd::Zero(3*nv, 3*nf);
  PHI_ = MatrixXd::Zero(3*nv, 3*ncv);
  auto &somigK = PSI_;
  auto &somigT = PHI_;
  #pragma omp parallel for
  for (int pid = 0; pid < V0_.cols(); ++pid) {

    const double A = 1/(4*M_PI), B = A/(4*(1-nu_));
    const point_t eta = V0_.col(pid);
    const point3d ETA(eta.x(), eta.y(), eta.z());    
    for (size_t f = 0; f < cageF_.cols(); ++f) {
      const size_t ix = cageF_(0, f), iy = cageF_(1, f), iz = cageF_(2, f);
      Vector3d v0 = cageV0_.col(ix);
      Vector3d v1 = cageV0_.col(iy);
      Vector3d v2 = cageV0_.col(iz);
      Vector3d  n = cageN0_.col(f);
      const double absA = (v1-v0).cross(v2-v0).norm();
      const double dt = (eta-v0).dot(n);

      point3d V0(v0.x(), v0.y(), v0.z()); 
      point3d V1(v1.x(), v1.y(), v1.z()); 
      point3d V2(v2.x(), v2.y(), v2.z()); 

      double h_psi_ = 0;
      mat33d bh_psi_H_;
      double h_psi0_ = 0; 
      double h_psi1_ = 0; 
      double h_psi2_ = 0;
      point3d h_psi0_grad_, h_psi1_grad_, h_psi2_grad_;
      mat33d h_psi0_H_, h_psi1_H_, h_psi2_H_;

      BiharmonicCoordinates3D::compute_Somigliana_coordinates(ETA, V0, V1, V2, h_psi_, bh_psi_H_, h_psi0_, h_psi1_, h_psi2_, h_psi0_grad_, h_psi1_grad_, h_psi2_grad_, h_psi0_H_, h_psi1_H_, h_psi2_H_);

      Matrix3d Bih_hes;
      {
        Bih_hes <<
            bh_psi_H_(0, 0), bh_psi_H_(0, 1), bh_psi_H_(0, 2),
            bh_psi_H_(1, 0), bh_psi_H_(1, 1), bh_psi_H_(1, 2),
            bh_psi_H_(2, 0), bh_psi_H_(2, 1), bh_psi_H_(2, 2);
      }
      Vector3d Bih_G_gra0, Bih_G_gra1, Bih_G_gra2;
      {
        Bih_G_gra0 << h_psi0_grad_[0], h_psi0_grad_[1], h_psi0_grad_[2];
        Bih_G_gra1 << h_psi1_grad_[0], h_psi1_grad_[1], h_psi1_grad_[2];
        Bih_G_gra2 << h_psi2_grad_[0], h_psi2_grad_[1], h_psi2_grad_[2];
      }
      Matrix3d Bih_G_hes0, Bih_G_hes1, Bih_G_hes2;
      {
        Bih_G_hes0 <<
            h_psi0_H_(0, 0), h_psi0_H_(0, 1), h_psi0_H_(0, 2),
            h_psi0_H_(1, 0), h_psi0_H_(1, 1), h_psi0_H_(1, 2),
            h_psi0_H_(2, 0), h_psi0_H_(2, 1), h_psi0_H_(2, 2);

        Bih_G_hes1 <<
            h_psi1_H_(0, 0), h_psi1_H_(0, 1), h_psi1_H_(0, 2),
            h_psi1_H_(1, 0), h_psi1_H_(1, 1), h_psi1_H_(1, 2),
            h_psi1_H_(2, 0), h_psi1_H_(2, 1), h_psi1_H_(2, 2);

        Bih_G_hes2 <<
            h_psi2_H_(0, 0), h_psi2_H_(0, 1), h_psi2_H_(0, 2),
            h_psi2_H_(1, 0), h_psi2_H_(1, 1), h_psi2_H_(1, 2),
            h_psi2_H_(2, 0), h_psi2_H_(2, 1), h_psi2_H_(2, 2);        
      }

      const Matrix3d &sgK = -B*8*M_PI*Bih_hes +4*M_PI*A*psi_(f, pid)*Matrix3d::Identity();
      somigK.block<3, 3>(3*pid, 3*f) = sgK;

      const Matrix3d &sgT0 = 4*M_PI*A*phi0(f, pid)*Matrix3d::Identity()
          +(A-2*B)*4*M_PI*(-Bih_G_gra0 *n.transpose()+n*Bih_G_gra0 .transpose())
          -2*B*dt*4*M_PI*Bih_G_hes0 ;      

      const Matrix3d &sgT1 = 4*M_PI*A*phi1(f, pid)*Matrix3d::Identity()
          +(A-2*B)*4*M_PI*(-Bih_G_gra1 *n.transpose()+n*Bih_G_gra1 .transpose())
          -2*B*dt*4*M_PI*Bih_G_hes1 ;

      const Matrix3d &sgT2 = 4*M_PI*A*phi2(f, pid)*Matrix3d::Identity()
          +(A-2*B)*4*M_PI*(-Bih_G_gra2 *n.transpose()+n*Bih_G_gra2 .transpose())
          -2*B*dt*4*M_PI*Bih_G_hes2 ;

      somigT.block<3, 3>(3*pid, 3*cageF_(0, f)) += sgT0;
      somigT.block<3, 3>(3*pid, 3*cageF_(1, f)) += sgT1;
      somigT.block<3, 3>(3*pid, 3*cageF_(2, f)) += sgT2;      
    }
  }

  // sanity check of PoU
  if ( sanity_check ) {
    double max_PoU_violation = 0;
    for (size_t i = 0; i < V0_.cols(); ++i) {
      Matrix3d sum = Matrix3d::Zero();
      for (size_t j = 0; j < cageV0_.cols(); ++j) { // for each edge
        sum.noalias() += somigT.block<3, 3>(3*i, 3*j);
      }
      double check_PoU = (sum-Matrix3d::Identity()).squaredNorm();
      if ( check_PoU > max_PoU_violation ) {
        max_PoU_violation = check_PoU;
      }
    }
    cout << "# max_pou_violation=" << max_PoU_violation << endl;
  }
}

void somig_deformer_3::deform(const vert_t &cageV, vert_t &meshV) {
  const size_t ncv = num_cage_vertices(), nv = num_mesh_vertices(),
      nf = num_cage_facets();

  // assign cage vertices
  for (size_t i = 0; i < cageV.size(); ++i) {
    cageV_(0, i) = cageV[i].x();
    cageV_(1, i) = cageV[i].y();
    cageV_(2, i) = cageV[i].z();    
  }

  calc_outward_normal();

  // this is global variant with no bulges
  typedef Eigen::Vector3d vec3d;
  mat3d gR; vec3d gT; double gS;
  {
    const auto &src = cageV0_;
    const auto &dst = cageV_;
      
    static const vec3d src_mean = src.rowwise().sum()/ncv;
    const vec3d dst_mean = dst.rowwise().sum()/ncv;
  
    static const Eigen::Matrix<double, 3, -1, Eigen::RowMajor> src_demean = src.colwise()-src_mean;
    Eigen::Matrix<double, 3, -1, Eigen::RowMajor> dst_demean = dst.colwise()-dst_mean;
      
    mat3d Cov = dst_demean*src_demean.transpose()/ncv, U, VT;
    vec3d diagS;
    lapacksvd<mat3d, vec3d>(Cov, U, VT, diagS);
       
    vec3d Sgn = vec3d::Ones();
    Sgn(2) = U.determinant()*VT.determinant() < 0 ? -1 : 1;
  
    gR = U*Sgn.asDiagonal()*VT;
    double src_var = src_demean.rowwise().squaredNorm().sum()/ncv;
    gS = 1.0/src_var*diagS.dot(Sgn);
    gT = dst_mean-gS*gR*src_mean;
  }
    
  dudn_.resize(3, nf);
  sf_.setZero(nf);
  for (size_t f = 0; f < dudn_.cols(); ++f) {
    double ss = gS, hh = ss;
    sf_[f] = 2*mu_*(hh+nu_/(1-2*nu_)*(2*ss+hh));
    dudn_.col(f) = sf_[f]*gR*cageN0_.col(f);
  }

  const mat3d gRT = gR.transpose();
  V_.reshaped().noalias() = PHI_*(gRT*cageV_).reshaped()+PSI_*(gRT*dudn_).reshaped();
  V_ = gR*V_;

  // retrieve mesh vertices
  for (size_t i = 0; i < meshV.size(); ++i) {
    meshV[i] = point3d(V_(0, i), V_(1, i), V_(2, i));
  }
}

void somig_deformer_3::calc_outward_normal() {
  if ( cageN_.size() == 0 ) {
    cageN_.resize(3, cageF_.cols());
  }
  
  for (size_t i = 0; i < cageF_.cols(); ++i) {
    const Eigen::Matrix<double, 3, 2> &es = cageV_(Eigen::all, cageF_.col(i).tail(2).array())-cageV_(Eigen::all, cageF_.col(i).head(2).array());
    cageN_.col(i) = (es.col(0).cross(es.col(1))).normalized();
  }
}
