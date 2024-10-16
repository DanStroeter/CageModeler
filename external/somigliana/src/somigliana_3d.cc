#include <somigliana/somigliana_3d.h>

#include <igl/read_triangle_mesh.h>
#include <boost/filesystem.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <CGAL/Weights/cotangent_weights.h>
#include <CGAL/Weights.h>
#include <igl/writeOBJ.h>

#include <somigliana/macro.h>
#include <somigliana/io.h>
#include <somigliana/kelvin_state.h>
#include <somigliana/timer.h>
#include <somigliana/util.h>
#include <somigliana/nanoflann.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


using namespace std;
using namespace Eigen;

namespace green {

typedef Eigen::Matrix2d mat2d;
typedef Eigen::Vector2d vec2d;

static inline mat3d cayley3(const mat3d &M) { 
  return (mat3d::Identity()-M)*(mat3d::Identity()+M).inverse();
}

template <typename T>
static int sign(T val) {
  return (T(0) < val) - (val < T(0));
}

static double solid_angle(const Vector3d &a,
                          const Vector3d &b,
                          const Vector3d &c) {
  typedef double T;
  
  T det = a.dot(b.cross(c));
  // if ( det < 0 ) {
  //   cerr << "# det(abc)=" << det << endl;
  // }
  if( fabs(det) < 0.0000000001 ) {
    return 2.0 * M_PI;
  }

  T al = a.norm(),   bl = b.norm(),   cl = c.norm();
  T div = al*bl*cl + a.dot(b)*cl + a.dot(c)*bl + b.dot(c)*al;
  T at = atan2( fabs(det) , div );
  //  if(at < 0) at += M_PI; // If det>0 && div<0 atan2 returns < 0, so add pi.
  T omega = 2.0 * at;
  return omega;
}
/*
extern "C" {
  void tet_linear_jac_(double *jac, const double *x, const double *Dm, const double *vol, const double *lam, const double *miu);
  void tet_linear_hes_(double *hes, const double *x, const double *Dm, const double *vol, const double *lam, const double *miu);  
}
*/
struct tet_fem
{
  mati_t tets_;
  size_t dim_;
  double mu_, lam_;
  VectorXd vol_;
  matd_t Dm_;

  tet_fem(const mati_t &tets, const matd_t &nods)
    : tets_(tets), dim_(nods.size()), mu_(1.0), lam_(0.0) {
    const size_t numT = tets.cols();
    vol_.resize(numT);
    Dm_.resize(9, numT);
    for (size_t i = 0; i < numT; ++i) {
      Matrix3d Dm = nods(Eigen::all, tets.col(i).tail(3).array()).colwise()-
          nods.col(tets(0, i));
      vol_[i] = fabs(Dm.determinant())/6.0;
      Dm_.col(i) = Dm.inverse().reshaped();
    }
  }
/*
  VectorXd Gra(const matd_t &x, const double nu) {
    lam_ = 2*nu/(1-2*nu);

    matd_t G = matd_t::Zero(3, dim_/3);
    for (size_t i = 0; i < tets_.cols(); ++i) {
      matd_t vert = x(Eigen::all, tets_.col(i).array()), grad = matd_t::Zero(3, 4);
      tet_linear_jac_(grad.data(), vert.data(), &Dm_(0, i), &vol_[i], &lam_, &mu_);
      G(Eigen::all, tets_.col(i).array()) += grad;
    }
    return G.reshaped();
  }

  SparseMatrix<double> Hes(const double nu) {
    lam_ = 2*nu/(1-2*nu);
    
    std::vector<Triplet<double>> trips;
    for (size_t i = 0; i < tets_.cols(); ++i) { 
      matd_t H = matd_t::Zero(12, 12);
      tet_linear_hes_(H.data(), nullptr, &Dm_(0, i), &vol_[i], &lam_, &mu_);
      for (size_t p = 0; p < 12; ++p) {
        for (size_t q = 0; q < 12; ++q) {
          const size_t I = 3*tets_(p/3, i)+p%3;
          const size_t J = 3*tets_(q/3, i)+q%3;
          trips.push_back(Triplet<double>(I, J, H(p, q)));
        }
      }      
    }
    SparseMatrix<double> HES(dim_, dim_);
    HES.setFromTriplets(trips.begin(), trips.end());
    return HES;
  }
  */
};

//===============================================================================
int somig_deformer_3::load_mesh(const string &file) {
  MatrixXi tmpF; matd_t tmpV;
  bool flag = igl::read_triangle_mesh(file, tmpV, tmpF);
  F_ = tmpF.transpose();
  V_ = tmpV.transpose();
  return flag;
}

int somig_deformer_3::load_cage(const Eigen::MatrixXd & vertices, const Eigen::MatrixXi & triangles)
{
  std::unordered_map<size_t, Surface_mesh::Vertex_index> v_it_map;

  // Adding vertices
  cageV_.resize(3, vertices.rows());
  for (int i = 0; i < vertices.rows(); ++i)
  {
    const Eigen::Vector3d vert = vertices.row(i);
    auto v_it = cage_.add_vertex(Point(vert(0), vert(1), vert(2)));
    v2i_.insert(std::make_pair(v_it, i));
    v_it_map.insert(std::make_pair(i, v_it));
    cageV_.col(i) = vert;
  }

  // Adding triangles
  cageF_.resize(3, triangles.rows());
  for (int i = 0; i < triangles.rows(); ++i)
  {
    const Eigen::Vector3i tri = triangles.row(i);
    auto f_it = cage_.add_face(v_it_map[tri(0)], v_it_map[tri(1)], v_it_map[tri(2)]);
    f2i_.insert(std::make_pair(f_it, i));
    cageF_.col(i) = tri;
  } 

  ASSERT(CGAL::is_triangle_mesh(cage_));

  return 0;
}

int somig_deformer_3::load_cage(const string &file) {
  ifstream input(file);
  if ( !input || !(input >> cage_) || cage_.is_empty() ) {
    //spdlog::error("input error: maybe the file is empty or not in *off* format.");
    return __LINE__;
  }

  ASSERT(CGAL::is_triangle_mesh(cage_));
  //spdlog::info("number of cells={}", cage_.number_of_faces());
  //spdlog::info("number of verts={}", cage_.number_of_vertices());
  //spdlog::info("number of edges={}", cage_.number_of_edges());

  // handle to index mapping
  size_t cnt = 0;
  for (Surface_mesh::Vertex_index v : cage_.vertices()) {
    v2i_.insert(std::make_pair(v, cnt++));
  }
  cnt = 0;
  for (Surface_mesh::Face_index f : cage_.faces()) {
    f2i_.insert(std::make_pair(f, cnt++));
  }

  // convert surface mesh to connectivity and coordinate matrices
  cageV_.resize(3, cage_.number_of_vertices());
  for (Surface_mesh::Vertex_index vi : cage_.vertices()) {
    const auto &p = cage_.point(vi);
    cageV_.col(v2i_[vi]) = Vector3d(p.x(), p.y(), p.z());
  }

  cageF_.resize(3, cage_.number_of_faces());
  for (Surface_mesh::Face_index fi : cage_.faces()) {
    const size_t fid = f2i_[fi];
    CGAL::Vertex_around_face_circulator<Surface_mesh>
        vcirc(cage_.halfedge(fi), cage_), done(vcirc);
    cageF_(0, fid) = *vcirc++;
    cageF_(1, fid) = *vcirc++;
    cageF_(2, fid) = *vcirc++;
    ASSERT(vcirc == done);
  }
  
  return 0;
}

void somig_deformer_3::init(const size_t num_quadrature) {
  ASSERT(cageV_.size() != 0);

  // record rest states
  {
    V0_ = V_;
    cageV0_ = cageV_;

    calc_outward_normal();
    cageN0_ = cageN_;

    calc_double_area();
    cageA0_ = cageA_;

    calc_vert_normal();
    cageNV0_ = cageNV_;
  }

  // compute initial volume
  volumeV0_ = 0;
  for (size_t f = 0; f < cageF_.cols(); ++f) {
    volumeV0_ += tri_sgn_vol<Vector3d>(cageV0_.col(cageF_(0, f)),
                                       cageV0_.col(cageF_(1, f)),
                                       cageV0_.col(cageF_(2, f)));
  }
  //spdlog::info("initial volume={}", volumeV0_);

  // sanity check
  const Vector3d glb_bc = cageV0_.rowwise().sum()/cageV0_.cols();
  Vector3d sum_lf = Vector3d::Zero(), sum_af = Vector3d::Zero();
  for (size_t f = 0; f < cageF_.cols(); ++f) {
    sum_lf += cageA0_(f)*cageN0_.col(f);
    Vector3d loc_bc = cageV0_(Eigen::all, cageF_.col(f).array()).rowwise().sum()/3.0;
    sum_af += cageA0_(f)*cageN0_.col(f).cross(loc_bc-glb_bc);
  }
  //spdlog::info("sanity check sum_lf={}, sum_af={}", sum_lf.norm(), sum_af.norm());

  trig_it_ = std::make_shared<trig_integrator>(3*num_quadrature*num_quadrature); // 2D triangular quadrature for purely numerical evaluation

  // CPU-GPU data transfer
  {
    // copy mesh infos
    const MatrixXi h_cageF = cageF_.cast<int>();
    const MatrixXf h_cageV = cageV0_.cast<float>();
    const MatrixXf h_cageN = cageN0_.cast<float>();
    const MatrixXf h_V     = V0_.cast<float>();
    cage_prec = std::make_shared<cage_precomputer>
        (cageF_.cols(), cageV_.cols(), V_.cols(),
         h_cageF.data(), h_cageV.data(), h_cageN.data(), h_V.data());

    // copy quadrature points and weights
    Matrix2Xf h_qp = trig_it_->qxy_.cast<float>();
    VectorXf  h_qw = trig_it_->qw_.cast<float>();
    //spdlog::info("total quadrature number={}", h_qw.size());
    //spdlog::info("sum of qw={}", h_qw.sum());
    cage_prec->copy_quadrature_to_device(h_qw.size(), h_qp.data(), h_qw.data());
  }

  // initial solid angle
  initSA_.resize(cageF_.cols());
  for (size_t f = 0; f < cageF_.cols(); ++f) {
    Vector3d q1 = cageNV0_.col(cageF_(0, f));
    Vector3d q2 = cageNV0_.col(cageF_(1, f));
    Vector3d q3 = cageNV0_.col(cageF_(2, f));
    initSA_[f] = solid_angle(q1, q2, q3);
  }

  // initial frames
  Frm0_.resize(cageF_.cols());
  for (size_t f = 0; f < cageF_.cols(); ++f) {
    auto &frm_r = Frm0_[f];
    
    frm_r.col(0) = (cageV0_.col(cageF_(1, f))-cageV0_.col(cageF_(0, f))).normalized();
    frm_r.col(2) = cageN0_.col(f);
    frm_r.col(1) = frm_r.col(2).cross(frm_r.col(0));
  }

  // initial tangent gradient
  Dm0_.resize(cageF_.cols());
  for (size_t f = 0; f < Dm0_.size(); ++f) {
    Dm0_[f].col(0) = Frm0_[f].leftCols(2).transpose()*(cageV0_.col(cageF_(1, f))-cageV0_.col(cageF_(0, f)));
    Dm0_[f].col(1) = Frm0_[f].leftCols(2).transpose()*(cageV0_.col(cageF_(2, f))-cageV0_.col(cageF_(0, f)));
    Dm0_[f] = Dm0_[f].inverse().eval();
  }
}

void somig_deformer_3::precompute_mvc_coords() {
  //spdlog::info("precompute MVC");
  const size_t ncv = num_cage_vertices(), nv = num_mesh_vertices(), nf = num_cage_facets();  
  //spdlog::info("ncv={}, nv={}, nf={}", ncv, nv, nf);

  high_resolution_timer clk;

  clk.start();
  const double TOL = 1e-6;
  
  Phi_ = matd_t::Zero(ncv, nv);
  #pragma omp parallel for
  for (size_t pid = 0; pid < nv; ++pid) {
    const Vector3d &x = V0_.col(pid);
    
    for (size_t i = 0; i < nf; ++i) {
      Eigen::Matrix3d u = cageV0_(Eigen::all, cageF_.col(i).array()).colwise()-x;
      Eigen::RowVector3d d = u.colwise().norm();
      u.colwise().normalize();
      const double sgn_u = sign(u.determinant());

      Vector3d l, theta, c, s;

      l[0] = (u.col(1)-u.col(2)).norm(),
      l[1] = (u.col(2)-u.col(0)).norm(),
      l[2] = (u.col(0)-u.col(1)).norm();

      theta[0] = 2*std::asin(l[0]/2);
      theta[1] = 2*std::asin(l[1]/2);
      theta[2] = 2*std::asin(l[2]/2);

      const double h = theta.sum()/2;
      
      if ( fabs(M_PI-h) < TOL ) {
        // x lies on t
        Phi_(cageF_(0, i), pid) += std::sin(theta[0])*d[1]*d[2];
        Phi_(cageF_(1, i), pid) += std::sin(theta[1])*d[2]*d[0];
        Phi_(cageF_(2, i), pid) += std::sin(theta[2])*d[0]*d[1];
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

        Phi_(cageF_(0, i), pid) += (theta[0]-c[2]*theta[1]-c[1]*theta[2])/(d[0]*sin(theta[2])*s[1]);
        Phi_(cageF_(1, i), pid) += (theta[1]-c[0]*theta[2]-c[2]*theta[0])/(d[1]*sin(theta[0])*s[2]);
        Phi_(cageF_(2, i), pid) += (theta[2]-c[1]*theta[0]-c[0]*theta[1])/(d[2]*sin(theta[1])*s[0]);
      }
    }
  }
  clk.stop();
  runtime = clk.duration()/1000.0;
  //spdlog::info("MVC comp time={}", runtime);
  
  // enforce PoU
  for (size_t j = 0; j < Phi_.cols(); ++j) {
    Phi_.col(j) /= Phi_.col(j).sum();
  }
}

void somig_deformer_3::precompute_green_coords() {
  //spdlog::info("precompute Green");
  const size_t ncv = num_cage_vertices(), nv = num_mesh_vertices(), nf = num_cage_facets();
  //spdlog::info("ncv={}, nv={}, nf={}", ncv, nv, nf);

  //  URAGO approach
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

  psi_.setZero(nf, nv);
  phi_.setZero(ncv, nv);

  #pragma omp parallel for
  for (size_t pid = 0; pid < V0_.cols(); ++pid) {
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
      for( unsigned int v = 0 ; v < 3 ; ++v ) phi_(cageF_(v, i), pid) += dot(Pt , J[v]) / (2.0 * At);
    }
  }

  ///> for translation invariance
  double max_pou_violation = 0;
  for (size_t j = 0; j < phi_.cols(); ++j) {
    const double col_violation = fabs(phi_.col(j).sum()-1.0);
    if ( col_violation > max_pou_violation ) {
      max_pou_violation = col_violation;
    }
  }
  //spdlog::info("maximum POU violation={}", max_pou_violation);  
}

void somig_deformer_3::precompute_somig_coords() {
  //spdlog::info("precompute Somig NM");
  const size_t ncv = num_cage_vertices(), nv = num_mesh_vertices(), ncf = num_cage_facets();

  high_resolution_timer clk;

  // computing on GPU
  MatrixXf h_PSI = MatrixXf::Zero(3*nv, 3*ncf);
  MatrixXf h_PHI = MatrixXf::Zero(3*nv, 3*ncv);
  clk.start();    
  cage_prec->precompute_somig(nu_, h_PHI.data(), h_PSI.data());
  clk.stop();  
  PHI_ = h_PHI.cast<double>();
  PSI_ = h_PSI.cast<double>();
  runtime = clk.duration()/1000.0;
  //spdlog::info("SOMIG comp time={}", runtime);
  
#if 1
  double max_PoU_violation = 0;
  for (size_t i = 0; i < V0_.cols(); ++i) {
    Matrix3d sum = Matrix3d::Zero();
    for (size_t j = 0; j < cageV0_.cols(); ++j) { // for each edge
      sum.noalias() += PHI_.block<3, 3>(3*i, 3*j);
    }
    double check_PoU = (sum-Matrix3d::Identity()).squaredNorm();
    if ( check_PoU > max_PoU_violation ) {
      max_PoU_violation = check_PoU;
    }
  }
  //spdlog::info("full-numerical max POU violation={}", sqrt(max_PoU_violation));
#endif  
}

void somig_deformer_3::precompute_phong_coords(const std::string &path) {
  //spdlog::info("precompute Phong");

  // generate tet based on cageF and cageV0
  if ( tet_mesh_read_from_vtk(path.c_str(), tetV0_, tetF_) ) {
    //spdlog::warn("no tet mesh for Phong deformation");
    return;
  }
  
  //spdlog::info("tets={}, nodes={}", tetF_.cols(), tetV0_.cols());
  //spdlog::info("tris={}, nodes={}", cageF_.cols(), cageV0_.cols());
  // there might be steiner points inserted
  ASSERT(tetV0_.cols() >= cageV0_.cols());
  ASSERT((tetV0_.leftCols(cageV0_.cols())-cageV0_).norm() < 1e-8*cageV0_.norm());

  // linear FEM on tet
  tet_fem_ = std::make_shared<tet_fem>(tetF_, tetV0_);
  {
    // selection matrix
    tetS_.resize(3*(tetV0_.cols()-cageV0_.cols()), 3*tetV0_.cols());
    for (size_t p = cageV0_.cols(); p < tetV0_.cols(); ++p) {
      tetS_.insert(3*(p-cageV0_.cols())+0, 3*p+0) = 1.0;
      tetS_.insert(3*(p-cageV0_.cols())+1, 3*p+1) = 1.0;
      tetS_.insert(3*(p-cageV0_.cols())+2, 3*p+2) = 1.0;      
    }
    if ( !tetS_.isCompressed() ) tetS_.makeCompressed();
  }

  const size_t n_tetv = tetV0_.cols(), n_tetf = tetF_.cols();

  // neighboring element of a node and bc
  tetV0_wk_.resize(n_tetv);
  tetBC0_.resize(3, n_tetf);  
  for (size_t f = 0; f < n_tetf; ++f) {
    tetBC0_.col(f) = tetV0_(Eigen::all, tetF_.col(f).array()).rowwise().sum()/4;
    
    tetV0_wk_[tetF_(0, f)].emplace_back(std::make_pair(f, 0.0));
    tetV0_wk_[tetF_(1, f)].emplace_back(std::make_pair(f, 0.0));
    tetV0_wk_[tetF_(2, f)].emplace_back(std::make_pair(f, 0.0));
    tetV0_wk_[tetF_(3, f)].emplace_back(std::make_pair(f, 0.0));
  }

  // neighboring weights of a node
  for (size_t v = 0; v < n_tetv; ++v) {
    const Vector3d tv = tetV0_.col(v);

    Matrix3d lhs = Matrix3d::Identity();
    Vector3d rhs = Vector3d::Zero();
    for (auto &f : tetV0_wk_[v]) { // neighboring tets
      Vector3d r = (tetBC0_.col(f.first)-tv).normalized();
      lhs += r*r.transpose();
      rhs -= r;
    }

    Vector3d lambda = lhs.llt().solve(rhs);

    for (auto &f : tetV0_wk_[v]) {
      Vector3d r = tetBC0_.col(f.first)-tv;
      f.second = (1.0+lambda.dot(r.normalized()))/r.norm();
    }
  }

  // build kd-tree
  using matrix_t = Eigen::Matrix<double, -1, -1>;
  using kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<matrix_t>;
  const matd_t trans_tet_bc = tetBC0_.transpose();
  kd_tree_t mat_index(3, std::cref(trans_tet_bc), 10);

  std::vector<Triplet<double>> trips;
  #pragma omp parallel for
  for (size_t pid = 0; pid < V0_.cols(); ++pid) {
    const Vector3d x = V0_.col(pid);

    const size_t numb = n_tetf; //std::min(size_t(5), nt);
    vector<size_t> ret_idx(numb);
    vector<double> dist_sqr(numb);
    nanoflann::KNNResultSet<double> res_set(numb);
    res_set.init(&ret_idx[0], &dist_sqr[0]);
    mat_index.index_->findNeighbors(res_set, x.data());

    // for each neighbors
    Vector3d w; size_t tid;
    for (int i = 0; i < numb; ++i) {
      tid = ret_idx[i];

      Vector3d
          v0 = tetV0_.col(tetF_(0, tid)),
          v1 = tetV0_.col(tetF_(1, tid)),
          v2 = tetV0_.col(tetF_(2, tid)),
          v3 = tetV0_.col(tetF_(3, tid));

      Matrix3d lhs;
      lhs.col(0) = v1-v0;
      lhs.col(1) = v2-v0;
      lhs.col(2) = v3-v0;
      w = lhs.lu().solve(x-v0);      
      if ( (w.array() >= 0.0).all() &&
           (w.array() <= 1.0).all() &&
           w.sum() >= 0.0 && w.sum() <= 1.0 ) {
        break;
      }
    }

    #pragma omp critical
    {
      trips.emplace_back(Triplet<double>(tetF_(0, tid), pid, 1.0-w.sum()));
      trips.emplace_back(Triplet<double>(tetF_(1, tid), pid, w.x()));
      trips.emplace_back(Triplet<double>(tetF_(2, tid), pid, w.y()));
      trips.emplace_back(Triplet<double>(tetF_(3, tid), pid, w.z()));
    }
  }

  phong_.resize(tetV0_.cols(), V0_.cols());  
  phong_.setFromTriplets(trips.begin(), trips.end());
}

void somig_deformer_3::deform(matd_t             &V,
                              const DeformerType &typeDf,
                              const BulgingType  &typeGamma,
                              const double       sigma,
                              const double       blend) {
  const size_t ncv = num_cage_vertices(), nv = num_mesh_vertices(),
      nf = num_cage_facets();

  // assign cage vertices
  cageV_ = V.topRows(ncv).transpose();

  if ( typeDf == PHONG ) {
    if ( phong_.size() == 0 ) {
      //spdlog::info("No embedding tets for this model!");
      return;
    }
    return;
    /*
    // displace tet nodes
    Matrix3Xd tetV = tetV0_;
    tetV.leftCols(cageV_.cols()) = cageV_;
    if ( tetV.cols() > cageV_.cols() ) { // there are free points
      const SparseMatrix<double> &LHS = tetS_*tet_fem_->Hes(nu_)*tetS_.transpose();
      const VectorXd &rhs = -tetS_*tet_fem_->Gra(tetV, nu_);
      SimplicialLLT<SparseMatrix<double>> slv;      
      slv.compute(LHS);
      ASSERT(slv.info() == Eigen::Success);
      tetV.reshaped() += tetS_.transpose()*slv.solve(rhs);
      ASSERT(slv.info() == Eigen::Success);
    }

    // compute deformation gradient per tet
    std::vector<Matrix3d> Ff(tetF_.cols());
    for (size_t f = 0; f < tetF_.cols(); ++f) {
      Eigen::Array3i idx_f = (tetF_.col(f).tail(3).array()).cast<int>();
      Ff[f] = (tetV(Eigen::all, idx_f).colwise()-tetV.col(tetF_(0, f)))*
          (tetV0_(Eigen::all, idx_f).colwise()-tetV0_.col(tetF_(0, f))).inverse();
    }

    // compute deformatin gradient per node
    std::vector<Matrix3d> Fv(tetV0_.cols());
    for (size_t v = 0; v < tetV0_.cols(); ++v) {
      Fv[v].setZero();
      double total = 0;
      for (const auto &p : tetV0_wk_[v]) {
        Fv[v] += p.second*Ff[p.first];
        total += p.second;
      }
      Fv[v] /= total;
    }

    V_.noalias() = 0.5*tetV*phong_;
    #pragma omp parallel for
    for (size_t v = 0; v < V_.cols(); ++v) {
      for (SparseMatrix<double>::InnerIterator it(phong_, v); it; ++it) {
        const size_t tv = it.row();
        V_.col(v) += 0.5*it.value()*(tetV.col(tv)+Fv[tv]*(V0_.col(v)-tetV0_.col(tv)));
      }
    }
    V.bottomRows(nv) = V_.transpose();

    return;
    */
  }
  
  if ( typeDf == MEANVALUE ) {
    V_.noalias() = cageV_*Phi_;
    //V.bottomRows(nv) = V_.transpose();
    return;    
  }

  calc_outward_normal();
  calc_double_area();

  if ( typeDf == GREEN ) {
    VectorXd sL = cageA_.cwiseQuotient(cageA0_);
    sL = sL.cwiseSqrt();
    V_.noalias() = cageV_*phi_ + (cageN_*sL.asDiagonal())*psi_;
    //V.bottomRows(nv) = V_.transpose();
    return;
  }
  
  high_resolution_timer clk;  

  if ( typeDf == SOMIGLIANA ) {
    clk.start();
    // Umemaya algorithm
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

    if ( typeGamma == SWEPT_VOLUME ) {
      // affine registered cage
      cageVr_.noalias() = gS*gR*cageV0_;
      cageVr_.colwise() += gT;
    }

    // calc vert normal
    calc_vert_normal();    
    
    // compute Rf
    Rf_.resize(nf);
    if ( fabs(blend) < 1e-6 ) { // global variant
      std::fill(Rf_.begin(), Rf_.end(), gR);
    } else {
      for (size_t f = 0; f < nf; ++f) {
        mat3d frm_p;
        {
          frm_p.col(0) = (cageV_.col(cageF_(1, f))-cageV_.col(cageF_(0, f))).normalized();
          frm_p.col(2) = cageN_.col(f);
          frm_p.col(1) = frm_p.col(2).cross(frm_p.col(0));
        }
        mat3d rf = frm_p*Frm0_[f].transpose();
        Rf_[f] = gR*cayley3(blend*cayley3(gR.transpose()*rf)+(1.0-blend)*mat3d::Zero());
      }
    }
        
    // compute Rv: cage node rotation
    Rv_.resize(ncv);
    if ( fabs(blend) < 1e-6 ) { // global variant
      std::fill(Rv_.begin(), Rv_.end(), gR);
    } else {
      std::fill(Rv_.begin(), Rv_.end(), mat3d::Zero());
      VectorXd valence = VectorXd::Zero(ncv);
      for (size_t f = 0; f < nf; ++f) {
        mat3d logRf = cayley3(gR.transpose()*Rf_[f]);
        Rv_[cageF_(0, f)] += cageA0_(f)*logRf;
        Rv_[cageF_(1, f)] += cageA0_(f)*logRf;
        Rv_[cageF_(2, f)] += cageA0_(f)*logRf;
        valence(cageF_.col(f).array()).array() += cageA0_(f);
      }
      for (size_t v = 0; v < ncv; ++v) {
        Rv_[v] = gR*cayley3(Rv_[v]/valence[v]);
      }
    }

    dudn_.resize(3, nf);
    sf_.setZero(nf);
    Matrix3Xd trac_d(3, nf);
    for (size_t f = 0; f < dudn_.cols(); ++f) {
      trac_d.col(f) = Rf_[f]*cageN0_.col(f);     

      mat2d Ds;
      Ds.col(0) = Frm0_[f].leftCols(2).transpose()*Rf_[f].transpose()*(cageV_.col(cageF_(1, f))-cageV_.col(cageF_(0, f)));
      Ds.col(1) = Frm0_[f].leftCols(2).transpose()*Rf_[f].transpose()*(cageV_.col(cageF_(2, f))-cageV_.col(cageF_(0, f)));
      mat2d tF = Ds*Dm0_[f];
      double ss = blend*0.5*sqrt((tF.transpose()*tF).trace()+2*cageA_(f)/cageA0_(f))+(1-blend)*gS;

      double hh = 0;
      if ( typeGamma == SOLID_ANGLE ) {
        Vector3d p1 = cageNV_.col(cageF_(0, f));
        Vector3d p2 = cageNV_.col(cageF_(1, f));
        Vector3d p3 = cageNV_.col(cageF_(2, f));
        double Ap = solid_angle(p1, p2, p3);
        hh = ss*exp(sigma*sigma*(Ap-initSA_[f])/(4*M_PI));
      } else {
        const auto i0 = cageF_(0, f), i1 = cageF_(1, f), i2 = cageF_(2, f);
        double Ve =
            tri_sgn_vol<vec3d>(cageV_.col(i0),
                               cageV_.col(i1),
                               cageV_.col(i2))+
            tri_sgn_vol<vec3d>(cageVr_.col(i2),
                               cageVr_.col(i1),
                               cageVr_.col(i0))+
            quad_sgn_vol<vec3d>(cageV_.col(i2),
                                cageV_.col(i1),
                                cageVr_.col(i1),
                                cageVr_.col(i2))+
            quad_sgn_vol<vec3d>(cageV_.col(i0),
                                cageV_.col(i2),
                                cageVr_.col(i2),
                                cageVr_.col(i0))+
            quad_sgn_vol<vec3d>(cageV_.col(i1),
                                cageV_.col(i0),
                                cageVr_.col(i0),
                                cageVr_.col(i1));                      
        hh = ss*exp(sigma*sigma*Ve/volumeV0_);
      } 
            
      sf_[f] = 2*mu_*(hh+nu_/(1-2*nu_)*(2*ss+hh));
      dudn_.col(f) = sf_[f]*trac_d.col(f);
    }

    if ( true ) {
      const auto &Ae = cageA0_;
      Matrix3Xd A = trac_d*Ae.asDiagonal();        
      VectorXd Binv = Ae.cwiseInverse();
      mat3d LHS = A*Binv.asDiagonal()*A.transpose();
      Vector3d z = LHS.llt().solve(A*sf_);
      sf_.noalias() -= Binv.asDiagonal()*A.transpose()*z;
      dudn_ = trac_d*sf_.asDiagonal();
    }

    if ( fabs(blend) < 1e-6 ) {
      const mat3d gRT = gR.transpose();
      V_.reshaped().noalias() = PHI_*(gRT*cageV_).reshaped()+PSI_*(gRT*dudn_).reshaped();
      V_ = gR*V_;      
    } else {
      #pragma omp parallel for
      for (size_t i = 0; i < nv; ++i) {
        mat3d sum_A = mat3d::Zero(), RAR;
        V_.col(i).setZero();
        for (size_t v = 0; v < cageV_.cols(); ++v) {
          RAR.noalias() = Rv_[v]*PHI_.block<3, 3>(3*i, 3*v)*Rv_[v].transpose();
          sum_A.noalias() += RAR;
          V_.col(i) += RAR*cageV_.col(v);
        }
        for (size_t f = 0; f < cageF_.cols(); ++f) {
          V_.col(i) += Rf_[f]*PSI_.block<3, 3>(3*i, 3*f)*sf_[f]*cageN0_.col(f);
        }
        V_.col(i) = sum_A.inverse()*V_.col(i);
      }
    }

    clk.stop();
    //spdlog::info("# editing time={}", clk.duration()/1000.0);

    // V.bottomRows(nv).noalias() = V_.transpose();
    return;
  }
  
  return;
}

void somig_deformer_3::calc_double_area() {
  if ( cageA_.size() == 0 ) {
    cageA_.resize(cageF_.cols());
  }

  for (size_t i = 0; i < cageF_.cols(); ++i) {
    const Eigen::Matrix<double, 3, 2> &es = cageV_(Eigen::all, cageF_.col(i).tail(2).array()).colwise()-cageV_.col(cageF_(0, i));
    cageA_[i] = (es.col(0).cross(es.col(1))).norm();
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

void somig_deformer_3::calc_vert_normal() {
  if ( cageNV_.size() == 0 ) {
    cageNV_.resize(3, cageV_.cols());
  }

  ASSERT(cageA0_.size() == cageF_.cols());

  const size_t nv = cageV_.cols();
  VectorXd areaV = VectorXd::Zero(nv);
  cageNV_.setZero(3, nv);
  for (size_t i = 0; i < cageF_.cols(); ++i) {
    Eigen::Matrix<double, 3, 2> es;
    es.col(0) = cageV_.col(cageF_(1, i))-cageV_.col(cageF_(0, i));
    es.col(1) = cageV_.col(cageF_(2, i))-cageV_.col(cageF_(1, i));
    const Vector3d dir = es.col(0).cross(es.col(1));

    cageNV_.col(cageF_(0, i)) += cageA0_[i]*dir;
    cageNV_.col(cageF_(1, i)) += cageA0_[i]*dir;
    cageNV_.col(cageF_(2, i)) += cageA0_[i]*dir;

    areaV[cageF_(0, i)] += cageA0_[i];
    areaV[cageF_(1, i)] += cageA0_[i];
    areaV[cageF_(2, i)] += cageA0_[i];    
  }

  for (size_t i = 0; i < cageNV_.cols(); ++i) {
    cageNV_.col(i) /= areaV[i];
    cageNV_.col(i).normalize();
  }
}

matd_t somig_deformer_3::barycenters() const {
  matd_t bc(cageF_.cols(), 3);
  for (size_t i = 0; i < cageF_.cols(); ++i) {
    bc.row(i) = (cageV_(Eigen::all, cageF_.col(i).array()).rowwise().sum().transpose())/3;
  }
  return bc;
}

int somig_deformer_3::save_cage(const char *file) const {
  tri_mesh_write_to_vtk(string(string(file)+".vtk").c_str(), cageV_, cageF_.cast<size_t>());
  return igl::writeOBJ(string(file)+".obj", cageV_.transpose(), cageF_.transpose());
}

int somig_deformer_3::save_register_cage(const char *file) const {
  tri_mesh_write_to_vtk(string(string(file)+".vtk").c_str(), cageVr_, cageF_.cast<size_t>());  
  return igl::writeOBJ(string(file)+".obj", cageVr_.transpose(), cageF_.transpose());
}

int somig_deformer_3::save_mesh(const char *file) const {
  tri_mesh_write_to_vtk(string(string(file)+".vtk").c_str(), V_, F_.cast<size_t>());  
  return igl::writeOBJ(string(file)+".obj", V_.transpose(), F_.transpose());
}

}
