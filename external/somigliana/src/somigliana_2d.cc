#include <somigliana/somigliana_2d.h>

#include <fstream>
#include <spdlog/spdlog.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <stack>

#include <somigliana/macro.h>
#include <somigliana/vtk.h>
#include <somigliana/kelvin_state.h>
#include <somigliana/quadrule.hpp>
#include <somigliana/util.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace Eigen;

namespace green {

std::tuple<Vector2d, double>
calc_centroid2d(const Eigen::MatrixXi &line, const matd_t &vertices) {
  Vector2d centroid = Vector2d::Zero();
  int vertexCount = vertices.cols();

  double signedArea = 0.0;  
  double x0 = 0.0; // Current vertex X
  double y0 = 0.0; // Current vertex Y
  double x1 = 0.0; // Next vertex X
  double y1 = 0.0; // Next vertex Y
  double a = 0.0;  // Partial signed area

  // For all vertices except last
  for (int p = 0; p < line.cols(); ++p) {
    int i = line(0, p), j = line(1, p);
    x0 = vertices.col(i).x();
    y0 = vertices.col(i).y();
    x1 = vertices.col(j).x();
    y1 = vertices.col(j).y();
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x() += (x0 + x1)*a;
    centroid.y() += (y0 + y1)*a;
  }

  signedArea *= 0.5;
  centroid.x() /= (6.0*signedArea);
  centroid.y() /= (6.0*signedArea);

  return {centroid, fabs(signedArea)};
}

static inline double cross2d(const vec2d &a, const vec2d &b) {
  return a.x()*b.y()-a.y()*b.x();
}

static inline mat2d cayley2d(const mat2d &M) {
  return (mat2d::Identity()-M)*(mat2d::Identity()+M).inverse();
}

static inline double angleX2Y(const Vector2d &a, const Vector2d &b) {
  return atan2(-cross2d(a, b), a.dot(b));
}

int somigliana_deformer2::load_mesh(const string &file) {
  ifstream input(file);
  if ( !input || !(input >> mesh_) || mesh_.is_empty() ) {
    spdlog::error("input error: maybe the file is empty or not in *off* format.");
    return __LINE__;
  }

  ASSERT(CGAL::is_triangle_mesh(mesh_));
  spdlog::info("number of cells={}", mesh_.number_of_faces());
  spdlog::info("number of verts={}", mesh_.number_of_vertices());

  size_t cnt = 0;
  for (Surface_mesh::Vertex_index v : mesh_.vertices()) {
    v2i_.insert(make_pair(v, cnt++));    
  }
  return 0;
}

int somigliana_deformer2::load_cage(const string &file) {
  ifstream input(file);
  if ( !input || !(input >> cage_) || cage_.is_empty() ) {
    spdlog::error("input error: maybe the file is empty or not in *off* format.");
    return __LINE__;
  }

  ASSERT(CGAL::is_triangle_mesh(cage_));
  spdlog::info("number of cells={}", cage_.number_of_faces());
  spdlog::info("number of verts={}", cage_.number_of_vertices());
  spdlog::info("number of edges={}", cage_.number_of_edges());
  
  return 0;
}

void somigliana_deformer2::init(const size_t num_quadrature) {
  spdlog::info("initialize somigliana defomer 2D");

  // compute Gauss quadratures
  const size_t nt = num_quadrature;
  qp_.resize(nt);
  qw_.resize(nt);
  cgqf(nt, 1, 0, 0, 0, 1, -1, &qp_[0], &qw_[0]);  
    
  {
    // handle mesh
    V_.resize(2, mesh_.number_of_vertices());  
    size_t vert_cnt = 0;
    for (Surface_mesh::Vertex_index vi : mesh_.vertices()) {
      Kernel::Point_3 pt = mesh_.point(vi);
      V_(0, vert_cnt) = pt.x();
      V_(1, vert_cnt) = pt.z();
      ++vert_cnt;
    }
    ASSERT(vert_cnt == mesh_.number_of_vertices());

    F_.resize(3, mesh_.number_of_faces());    
    size_t face_cnt = 0;
    for (Surface_mesh::Face_index fi : mesh_.faces()) {
      CGAL::Vertex_around_face_circulator<Surface_mesh>
          vcirc(mesh_.halfedge(fi), mesh_), done(vcirc);

      F_(0, face_cnt) = *vcirc++;
      F_(1, face_cnt) = *vcirc++;
      F_(2, face_cnt) = *vcirc++;
      ASSERT(vcirc == done);

      ++face_cnt;
    }
    ASSERT(face_cnt == mesh_.number_of_faces());
    spdlog::info("done initializing mesh...");
  }

  // handle cage
  {
    cageV_.resize(2, cage_.number_of_vertices());
    size_t vert_cnt = 0;
    for (Surface_mesh::Vertex_index vi : cage_.vertices()) {
      Kernel::Point_3 pt = cage_.point(vi);
      cageV_(0, vert_cnt) = pt.x();
      cageV_(1, vert_cnt) = pt.z();
      ++vert_cnt;
    }
    ASSERT(vert_cnt == cage_.number_of_vertices());

    cageF_.resize(3, cage_.number_of_faces());    
    size_t face_cnt = 0;
    for (Surface_mesh::Face_index fi : cage_.faces()) {
      CGAL::Vertex_around_face_circulator<Surface_mesh>
          vcirc(cage_.halfedge(fi), cage_), done(vcirc);

      cageF_(0, face_cnt) = *vcirc++;
      cageF_(1, face_cnt) = *vcirc++;
      cageF_(2, face_cnt) = *vcirc++;
      ASSERT(vcirc == done);

      ++face_cnt;
    }
    ASSERT(face_cnt == cage_.number_of_faces());    

    // boundary edges
    std::vector<size_t> cageE;
    for (Surface_mesh::Halfedge_index hi : cage_.halfedges()) {
      if ( cage_.face(hi) == Surface_mesh::null_face() ) {
        // is a boundary edge
        cageE.emplace_back(cage_.source(hi));
        cageE.emplace_back(cage_.target(hi));
      }
    }
    spdlog::info("boundary edge number={}", cageE.size()/2);

    cageE_.resize(2, cageE.size()/2);
    std::copy(cageE.begin(), cageE.end(), cageE_.data());
    cout << cageE_ << endl;
  }
  
  {
    V0_ = V_;
    cageV0_ = cageV_;
  
    // get rest segment element length
    calc_edge_length();
    cageL0_ = cageL_;

    // calc initial facet normal
    calc_outward_normal();
    cageN0_ = cageN_;

    // calc initial nodal normal
    calc_vert_normal();
    cageVN0_ = cageVN_;
  }
  spdlog::info("Cage xmin={}, xmax={}", cageV0_.row(0).minCoeff(), cageV0_.row(1).maxCoeff());
  spdlog::info("Cage ymin={}, ymax={}", cageV0_.row(1).minCoeff(), cageV0_.row(1).maxCoeff());

  // reorder node index via DFS
  spdlog::info("reorder vertex indices");
  {
    const size_t nv = cageV_.cols();
    const size_t ne = cageE_.cols();

    // build adjancency graph
    vector<size_t> u(ne), v(ne), first(nv, -1), next(ne, -1);
    {
      for (size_t i = 0; i < cageE_.cols(); ++i) {
        size_t eid = i;
        u[eid] = cageE_(0, i);
        v[eid] = cageE_(1, i);
        next[eid] = first[u[eid]];
        first[u[eid]] = eid;
      }
    }

    vector<bool> vis(nv, false);    
    std::stack<size_t> q;
    q.push(0);
    while ( !q.empty() ) {
      const size_t curr = q.top();
      q.pop();
      if ( !vis[curr] ) {
        vis[curr] = true;
        cageV_loop_.push_back(curr);
      }

      for (size_t e = first[curr]; e != -1; e = next[e]) {
        size_t adj_v = v[e];
        if ( !vis[adj_v] ) {
          q.push(adj_v);
        }              
      }
    }
    cout << "cage node number=" << cageV_loop_.size() << endl << "vertices in loop: " << endl;
    for (auto &idx : cageV_loop_) {
      cout << idx << " ";
    }
    cout << endl;
  }

  vec2d bc0;
  std::tie(bc0, areaA0_) = calc_centroid2d(cageE_, cageV0_);
  spdlog::info("rest cage area={}", areaA0_);

  // sanity check
  vec2d sum_le = vec2d::Zero();
  for (size_t e = 0; e < cageE_.cols(); ++e) {
    sum_le += cageL0_(e)*cageN0_.col(e);    
  }
  ASSERT(sum_le.squaredNorm() < 1e-12);

  // initial solid angle
  initTA_.resize(cageE_.cols());
  for (size_t e = 0; e < cageE_.cols(); ++e) {
    vec2d nq1 = cageVN0_.col(cageE_(0, e));
    vec2d nq2 = cageVN0_.col(cageE_(1, e));
    initTA_[e] = angleX2Y(nq1, nq2);
  }
  prevTA_ = initTA_;
}

void somigliana_deformer2::calc_edge_length() {
  cageL_.resize(cageE_.cols());
  for (size_t i = 0; i < cageE_.cols(); ++i) {
    cageL_[i] = (cageV_.col(cageE_(0, i))-cageV_.col(cageE_(1, i))).norm();
  }
}

void somigliana_deformer2::calc_outward_normal() {
  cageN_.resize(2, cageE_.cols());
  for (size_t i = 0; i < cageE_.cols(); ++i) {
    Vector2d dir = cageV_.col(cageE_(1, i))-cageV_.col(cageE_(0, i));
    dir /= dir.norm();
    cageN_(0, i) = -dir[1];
    cageN_(1, i) = dir[0];
  }
}

void somigliana_deformer2::calc_vert_normal() {
  const size_t nv = cageV_.cols();
  
  cageVN_ = matd_t::Zero(2, nv);
  VectorXd vl = VectorXd::Zero(nv);
  for (size_t e = 0; e < cageE_.cols(); ++e) {
    Vector2d dir = cageV_.col(cageE_(1, e))-cageV_.col(cageE_(0, e));
    std::swap(dir[0], dir[1]);
    dir(0) *= -1;
    
    const double len = (cageV0_.col(cageE_(1, e))-cageV0_.col(cageE_(0, e))).norm();
    cageVN_.col(cageE_(0, e)) += len*dir;
    cageVN_.col(cageE_(1, e)) += len*dir;
    vl(cageE_(0, e)) += len;
    vl(cageE_(1, e)) += len;
  }

  for (size_t v = 0; v < nv; ++v) {
    cageVN_.col(v) /= vl(v);
    cageVN_.col(v).normalize();
  }
}

void somigliana_deformer2::precompute_meanv_coords() {
  spdlog::info("precompute MVC");
  const size_t ncv = cageV_.cols(), nv = V_.cols();

  Phi_ = matd_t::Zero(ncv, nv);
  #pragma omp parallel for
  for (int i = 0; i < nv; ++i) { // for each mesh point
    const Vector2d &x = V_.col(i);

    for (int e = 0; e < cageE_.cols(); ++e) {
      const Vector2d &e0 = cageV_.col(cageE_(0, e))-x;
      const Vector2d &e1 = cageV_.col(cageE_(1, e))-x;

      const double a = std::atan2(e1.y(), e1.x())-std::atan2(e0.y(), e0.x());
      const double tan_half_a = std::tan(a/2);

      Phi_(cageE_(0, e), i) += tan_half_a/e0.norm();
      Phi_(cageE_(1, e), i) += tan_half_a/e1.norm();
    }

    Phi_.col(i) /= Phi_.col(i).sum();
  }
}

void somigliana_deformer2::precompute_green_coords() {
  spdlog::info("precompute Green");
  const int d = 2;
  phi_ = matd_t::Zero(d*V_.cols(), d*cageV_.cols());
  psi_ = matd_t::Zero(d*V_.cols(), d*cageN_.cols());

  const mat2d &Id = mat2d::Identity();

  #pragma omp parallel for
  for (size_t pid = 0; pid < V_.cols(); ++pid) {
    for (size_t i = 0; i < cageE_.cols(); ++i) {
      //-> cage node to mesh node mapping
      Vector2d a = cageV_.col(cageE_(1, i))-cageV_.col(cageE_(0, i));
      Vector2d b = cageV_.col(cageE_(0, i))-V_.col(pid);
      double Q, S, R, BA, SRT, L0, L1, A0, A1, A10, L10, psi_entry, phi_entry1, phi_entry0;
      Q = a.dot(a);
      S = b.dot(b);
      R = 2*a.dot(b);
      BA = b.dot(a.norm()*cageN_.col(i));
      SRT = sqrt(4*S*Q-R*R);
      L0 = log(S);
      L1 = log(S+Q+R);
      A0 = atan2(R, SRT)/SRT;
      A1 = atan2(2*Q+R, SRT)/SRT;
      A10 = A1 - A0;
      L10 = L1 - L0;
      psi_entry = -a.norm()/(4*M_PI)*((4*S-R*R/Q)*A10 + R/(2*Q)*L10 + L1 - 2);
      phi_entry1 = -BA/(2*M_PI)*(L10/(2*Q) - A10*R/Q);
      phi_entry0 = +BA/(2*M_PI)*(L10/(2*Q) - A10*(2+R/Q));

      #pragma omp critical
      {
        psi_.block<2, 2>(2*pid, 2*i) += psi_entry*Id;
        phi_.block<2, 2>(2*pid, 2*cageE_(1, i)) += -phi_entry1*Id;
        phi_.block<2, 2>(2*pid, 2*cageE_(0, i)) += -phi_entry0*Id;
      }
    }
  }
  
#if 0
  // check phi: parition to unity
  for (size_t i = 0; i < V_.cols(); ++i) {
    mat2d sum = mat2d::Zero();
    for (size_t j = 0; j < cageV_.cols(); ++j) {
      sum += phi_.block<2, 2>(2*i, 2*j);
    }
    std::cout << sum << std::endl << std::endl;
  }

  // check psi: ?
  for (size_t i = 0; i < V_.cols(); ++i) {
    mat2d sum = mat2d::Zero();
    for (size_t j = 0; j < cageE_.cols(); ++j) {
      sum += psi_.block<2, 2>(2*i, 2*j);
    }
    std::cout << sum << std::endl << std::endl;
  }
#endif
}

void somigliana_deformer2::precompute_somig_coords() {
  spdlog::info("precompute SOMIG numerically");
  const int d = 2;
  matd_t PHI = matd_t::Zero(d*V0_.cols(), d*cageV0_.cols());
  matd_t PSI = matd_t::Zero(d*V0_.cols(), d*cageN0_.cols());

  const size_t ncv = cageV0_.cols(), nv = V0_.cols(), nf = cageN0_.cols();

  spdlog::info("mu={}, nu={}", mu_, nu_);
  const double C1 = 1/(8*M_PI*mu_*(1-nu_)), C2 = -1/(4*M_PI*(1-nu_));
  const mat2d &Id = mat2d::Identity();

  // kelvin kernel
  kelvin_displ kd(mu_, nu_);
  kelvin_traction kt(mu_, nu_);

  #pragma omp parallel for
  for (size_t pid = 0; pid < V0_.cols(); ++pid) {
    for (size_t i = 0; i < cageE_.cols(); ++i) {
      //-> cage node to mesh node mapping
      const Vector2d e = cageV0_.col(cageE_(1, i))-cageV0_.col(cageE_(0, i));
      const Vector2d d = cageV0_.col(cageE_(0, i))-V0_.col(pid);
      const Vector2d n = cageN0_.col(i);
      const double absE = e.norm();

      // local numerical integration
      mat2d IK = mat2d::Zero();
      mat2d IT1 = mat2d::Zero();
      mat2d IT2 = mat2d::Zero();
      for (size_t k = 0; k < qw_.size(); ++k) {
        const double t = qp_[k], w = qw_[k];
        const Vector2d &r = e*t+d;        
        IK += kd.K(r)*w;
        IT1 += kt.T(r, n)*-(1-t)*w;
        IT2 += kt.T(r, n)*-t*w;
      }

      PSI.block<2, 2>(2*pid, 2*i) = absE*IK;
      PHI.block<2, 2>(2*pid, 2*cageE_(0, i)) += absE*IT1;
      PHI.block<2, 2>(2*pid, 2*cageE_(1, i)) += absE*IT2;
    }
  }
  
  // assign 
  PSI_ = PSI;
  PHI_ = PHI;
  
#if 1
  // check phi: parition to unity
  double max_PoU_violation = 0;
  for (size_t i = 0; i < V0_.cols(); ++i) {
    mat2d sum = mat2d::Zero();
    for (size_t j = 0; j < cageV0_.cols(); ++j) { // for each edge
      sum += PHI_.block<2, 2>(2*i, 2*j);
    }
    double check_PoU = (sum-mat2d::Identity()).squaredNorm();
    if ( check_PoU > max_PoU_violation ) {
      max_PoU_violation = check_PoU;
    }
  }
  spdlog::info("Somig(NM) max PoU violation={}", sqrt(max_PoU_violation));
#endif
}

double somigliana_deformer2::mesh_area() const {
  double total_area = 0;
  for (size_t i = 0; i < F_.cols(); ++i) {
    double l[3];
    for (size_t j = 0; j < 3; ++j) {
      l[j] = (V_.col(F_(j, i))-V_.col(F_((j+1)%3, i))).norm();
    }
    const double s = 0.5*(l[0]+l[1]+l[2]);
    total_area += sqrt(s*(s-l[0])*(s-l[1])*(s-l[2]));
  }
  return total_area;
}

void somigliana_deformer2::deform(matd_t             &V,
                                  const DeformerType &typeDf,
                                  const BulgingType  &typeGamma,
                                  const double       sigma,
                                  const double       blend,
                                  const plot_info    &plt,
                                  matd_t             &dx,
                                  matd_t             &dy,
                                  matd_t             &bc,
                                  matd_t             &trac) {
  const size_t ncv = cageV_.cols(), nv = V_.cols(), ne = cageE_.cols();
  mat2d R90;
  R90 << 0, -1, 1, 0;
    
  // assign cageV
  for (size_t i = 0; i < ncv; ++i) {
    cageV_(0, i) = V(i, 0);
    cageV_(1, i) = V(i, 2);
  }
    
  if ( typeDf == MEANVALUE ) {
    dx.resize(0, 0);
    dy.resize(0, 0);
    V_.noalias() = cageV_*Phi_;
    for (size_t i = 0; i < nv; ++i) {
      V(i+ncv, 0) = V_(0, i);
      V(i+ncv, 2) = V_(1, i);
    }
    return;
  }

  calc_outward_normal();
  calc_edge_length();  
  
  if ( typeDf == GREEN ) {
    dx.resize(0, 0);
    dy.resize(0, 0);

    // normal derivative per boundary edge
    Matrix2Xd dudn = Matrix2Xd::Zero(2, cageN0_.cols());
    for (size_t e = 0; e < dudn.cols(); ++e) {
      dudn.col(e) = cageL_(e)/cageL0_(e)*cageN_.col(e);
    }

    // deform
    V_.reshaped().noalias() = phi_*cageV_.reshaped()+psi_*dudn.reshaped();

    for (size_t i = 0; i < nv; ++i) {
      V(i+ncv, 0) = V_(0, i);
      V(i+ncv, 2) = V_(1, i);
    }
    return;
  }
  
  if ( typeDf == SOMIGLIANA ) {
    // Umemaya algorithm
    mat2d gR; vec2d gT; double gS;
    {
      const auto &src = cageV0_;
      const auto &dst = cageV_;
      
      const vec2d src_mean = src.rowwise().sum()/ncv;
      const vec2d dst_mean = dst.rowwise().sum()/ncv;
  
      Eigen::Matrix<double, 2, -1, Eigen::RowMajor> src_demean, dst_demean;
      src_demean = src.colwise()-src_mean;
      dst_demean = dst.colwise()-dst_mean;
      
      mat2d Cov = dst_demean*src_demean.transpose()/ncv, U, VT;
      vec2d diagS;
      lapacksvd<mat2d, vec2d>(Cov, U, VT, diagS);
       
      vec2d Sgn = vec2d::Ones();      
      Sgn(1) = U.determinant()*VT.determinant() < 0 ? -1 : 1;
  
      gR = U*Sgn.asDiagonal()*VT;
      double src_var = src_demean.rowwise().squaredNorm().sum()/ncv;
      gS = 1.0/src_var*diagS.dot(Sgn);
      gT = dst_mean-gS*gR*src_mean;
    }

    if ( typeGamma == SWEPT_VOLUME ) {
      // affine registered cage
      cageVr_.noalias() = gS*gR*cageV0_;
      cageVr_.colwise() += gT;
    }

    // current vertex normal
    calc_vert_normal();

    // Re: edge rotation
    Re_.resize(ne);
    for (size_t e = 0; e < ne; ++e) {
      const vec2d &n0 = cageN0_.col(e);
      const vec2d &n  = cageN_.col(e);
      const double a = atan2(n(1), n(0))-atan2(n0(1), n0(0));
      mat2d re; // a local frame
      re << cos(a), -sin(a), sin(a), cos(a);
      Re_[e] = gR*cayley2d(blend*cayley2d(gR.transpose()*re)+(1.0-blend)*mat2d::Zero());
    }

    // Rv: cage node rotation
    Rv_.resize(ncv);
    for (size_t v = 0; v < ncv; ++v) {
      const vec2d &n0 = cageVN0_.col(v);
      const vec2d &n  = cageVN_.col(v);
      const double a = atan2(n(1), n(0))-atan2(n0(1), n0(0));
      mat2d rv; // a local frame
      rv << cos(a), -sin(a), sin(a), cos(a);
      Rv_[v] = gR*cayley2d(blend*cayley2d(gR.transpose()*rv)+(1.0-blend)*mat2d::Zero());
    }
    
    // plot frame field
    {
      // assemble co-rotated frames
      if ( dx.size() == 0 || dy.size() == 0 ) {
        dx.resize(V.rows(), V.cols());
        dy.resize(V.rows(), V.cols());
      }
      
      dx.setZero();
      dy.setZero();
      
      const int nce = cageE_.cols();
      const int EID = std::min(std::max(plt.eid_, 0), static_cast<int>(ncv+nce-1));
      #pragma omp parallel for
      for (size_t i = 0; i < nv; ++i) {
        mat2d C;
        if (EID < ncv) C = PHI_.block<2, 2>(2*i, 2*EID);
        else C = PSI_.block<2, 2>(2*i, 2*(EID-ncv));

        vec2d dn, dt;
        dn = (EID < ncv) ? cageVN0_.col(EID) : cageN0_.col(EID-ncv);
        dt = -R90*dn;

        mat2d F;
        F.col(0) = C*dt;
        F.col(1) = C*dn;
        
        dx(i+ncv, 0) = F(0, 0); dx(i+ncv, 1) = 0; dx(i+ncv, 2) = F(1, 0);
        dy(i+ncv, 0) = F(0, 1); dy(i+ncv, 1) = 0; dy(i+ncv, 2) = F(1, 1);
      }

      double max_x = dx.rowwise().norm().maxCoeff();
      double max_y = dy.rowwise().norm().maxCoeff();
      dx /= std::max(max_x, max_y);
      dy /= std::max(max_x, max_y);
    }

    V.col(0).tail(nv).setZero();
    V.col(2).tail(nv).setZero();
    
    dudn_.resize(2, ne);
    se_.setZero(ne);
    Matrix2Xd trac_d(2, ne);
    for (size_t e = 0; e < dudn_.cols(); ++e) {
      trac_d.col(e) = Re_[e]*cageN0_.col(e);
      
      double ss = blend*cageL_(e)/cageL0_(e)+(1-blend)*gS;

      double hh = 0;
      if ( typeGamma == SOLID_ANGLE ) {
        vec2d np1 = cageVN_.col(cageE_(0, e));
        vec2d np2 = cageVN_.col(cageE_(1, e));
        double currTA = angleX2Y(np1, np2);
        double diff = currTA-initTA_(e);
        if ( diff > M_PI ) {
          currTA -= 2*M_PI;
        } else if ( diff < -M_PI ) {
          currTA += 2*M_PI;
        }
        hh = ss*exp(sigma*sigma*(currTA-initTA_[e])/(2*M_PI));
      } else {            
        const double Ae = 
            quad_area_2d<vec2d>(
                cageV_.col(cageE_(1, e)),
                cageV_.col(cageE_(0, e)),
                cageVr_.col(cageE_(0, e)),
                cageVr_.col(cageE_(1, e)));
        hh = ss*exp(sigma*sigma*Ae/areaA0_);
      }

      se_[e] = 2*mu_*(hh+nu_/(1-2*nu_)*(ss+hh));
      dudn_.col(e) = se_[e]*trac_d.col(e);
    }

    if ( true ) {
      const auto &le = cageL0_;
      Matrix2Xd A = trac_d*le.asDiagonal();
      VectorXd Binv = le.cwiseInverse();
      mat2d LHS = A*Binv.asDiagonal()*A.transpose();
      vec2d z = LHS.llt().solve(A*se_);
      se_.noalias() -= Binv.asDiagonal()*A.transpose()*z;
      dudn_.noalias() = trac_d*se_.asDiagonal();
    }
    
    // write out edge centers and traction vectors
    if ( 1 ) {
      // normal derivative per boundary edge
      Matrix2Xd mid_e(2, ne), mid_e0(2, ne);
      for (size_t e = 0; e < ne; ++e) {
        mid_e.col(e) = cageV_(Eigen::all, cageE_.col(e).array()).rowwise().sum()/2.0;
        mid_e0.col(e) = cageV0_(Eigen::all, cageE_.col(e).array()).rowwise().sum()/2.0;
      }    
      
      bc.setZero(ne+ncv, 3);
      trac.setZero(ne+ncv, 3);

      bc.col(0).head(ne) = mid_e.row(0).transpose();
      bc.col(2).head(ne) = mid_e.row(1).transpose();
      bc.col(0).tail(ncv) = cageV_.row(0).transpose();
      bc.col(2).tail(ncv) = cageV_.row(1).transpose();

      trac.col(0).head(ne) = dudn_.row(0).transpose();
      trac.col(2).head(ne) = dudn_.row(1).transpose();
      trac.col(0).tail(ncv) = cageVN_.row(0).transpose();
      trac.col(2).tail(ncv) = cageVN_.row(1).transpose();
    }
    
    #pragma omp parallel for
    for (size_t i = 0; i < V_.cols(); ++i) {
      mat2d sum_A = mat2d::Zero(), RAR;
      V_.col(i).setZero();
      for (size_t v = 0; v < cageV_.cols(); ++v) {
        RAR.noalias() = Rv_[v]*PHI_.block<2, 2>(2*i, 2*v)*Rv_[v].transpose();
        sum_A.noalias() += RAR;
        V_.col(i) += RAR*cageV_.col(v);
      }
      for (size_t e = 0; e < cageE_.cols(); ++e) {
        V_.col(i) += Re_[e]*(PSI_.block<2, 2>(2*i, 2*e)*se_[e]*cageN0_.col(e));
      }
      // to ensure partition of unity
      V_.col(i) = sum_A.inverse()*V_.col(i);
    }

    V.col(0).tail(nv) = V_.row(0).transpose();
    V.col(2).tail(nv) = V_.row(1).transpose();
    return;
  }
  
  return;
}

int somigliana_deformer2::save_mesh(const string &file) {
  matd_t nods_3d = matd_t::Zero(3, V_.cols());
  nods_3d.row(0) = V_.row(0);
  nods_3d.row(2) = V_.row(1);

  ofstream os(file+".vtk");
  if ( os.fail() ) {
    spdlog::error("error in output file");
    return __LINE__;
  }
  tri2vtk(os, nods_3d.data(), nods_3d.cols(), F_.data(), F_.cols());
  os.close();

  string obj_file(file+".obj");
  igl::writeOBJ(obj_file, nods_3d.transpose(), F_.transpose(), RowVector3d(0, 1, 0).replicate(V_.cols(), 1), F_.transpose(), V0_.transpose(), F_.transpose());

  return 0;
}

int somigliana_deformer2::save_cage(const string &file) {
  matd_t cageV_3d = matd_t::Zero(3, cageV_.cols());
  cageV_3d.row(0) = cageV_.row(0);
  cageV_3d.row(2) = cageV_.row(1);

  ofstream os(file+".vtk");
  if ( os.fail() ) {
    spdlog::error("error in output file");
    return __LINE__;
  }
  line2vtk(os, cageV_3d.data(), cageV_3d.cols(), cageE_.data(), cageE_.cols());
  os.close();

  string obj_file(file+".obj");
  igl::writeOBJ(obj_file, cageV_3d.transpose(), cageF_.transpose());

  return 0;
}

int somigliana_deformer2::save_register_cage(const string &file) {
  matd_t cageV_3d = matd_t::Zero(3, cageV_.cols());
  cageV_3d.row(0) = cageVr_.row(0);
  cageV_3d.row(2) = cageVr_.row(1);

  ofstream os(file+".vtk");
  if ( os.fail() ) {
    spdlog::error("error in output file");
    return __LINE__;
  }
  line2vtk(os, cageV_3d.data(), cageV_3d.cols(), cageE_.data(), cageE_.cols());
  os.close();

  string obj_file(file+".obj");
  igl::writeOBJ(obj_file, cageV_3d.transpose(), cageF_.transpose());

  return 0;
}

int somigliana_deformer2::save_normal(const string &file) {
  mati_t L(2, cageE_.cols());
  matd_t v(2, 2*cageE_.cols());
  for (size_t i = 0; i < cageE_.cols(); ++i) {
    L(0, i) = 2*i+0;
    L(1, i) = 2*i+1;
    v.col(2*i+0) = 0.5*(cageV_.col(cageE_(0, i))+cageV_.col(cageE_(1, i)));
    v.col(2*i+1) = v.col(2*i+0)+cageN_.col(i);
  }

  matd_t V = matd_t::Zero(3, v.cols());
  V.row(0) = v.row(0);
  V.row(2) = v.row(1);

  ofstream os(file);
  if ( os.fail() ) {
    spdlog::error("no output file");
    return __LINE__;
  }
  line2vtk(os, V.data(), V.cols(), L.data(), L.cols());
  os.close();
  
  return 0;
}

}
