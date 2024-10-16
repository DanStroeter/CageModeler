#ifndef SOMIGLIANA_3D_H
#define SOMIGLIANA_3D_H

#include <memory>
#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <Eigen/Sparse>

#include "types.h"
#include "green_core.h"
#include "trig_quad_rule.h"

namespace green {

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3  Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;
typedef Eigen::Matrix3d mat3d;

//=========== Gauss-Lengedre integrator ========================
struct trig_integrator
{
  // quadrature points and weights
  Eigen::Matrix2Xd qxy_;
  Eigen::VectorXd  qw_;

  trig_integrator(const size_t num_quadrature) {
    sym_trig_quad_rule(num_quadrature, qxy_, qw_);
  }
};

//==============================================================
struct tet_fem;

class somig_deformer_3
{
 public:
  somig_deformer_3(bool quiet = false)
      : mu_(1.0), nu_(0.0), runtime(0) {
        if (quiet) {
          spdlog::set_level(spdlog::level::warn);
        }
      }
  somig_deformer_3(const double nu, bool quiet = false)
      : mu_(1.0), nu_(nu), runtime(0) {
        if (quiet) {
          spdlog::set_level(spdlog::level::warn);
        }
      }

  void set_nu(const double nu) {
    nu_ = nu;
  }
  double get_nu() const { return nu_; }

  matd_t & getPhi() { return Phi_; }

  int load_mesh(const std::string &file);
  int load_cage(const std::string &file);
  int load_cage(const Eigen::MatrixXd & vertices, const Eigen::MatrixXi & triangles);
  void init(const size_t num_quadrature=32);

  size_t num_cage_vertices() const { return cageV_.cols(); }
  size_t num_mesh_vertices() const { return V_.cols(); }
  size_t num_cage_facets() const { return cageF_.cols(); }

  void precompute_mvc_coords();
  void precompute_green_coords();
  void precompute_somig_coords();
  void precompute_bem();
  void precompute_phong_coords(const std::string &path);

  void deform(matd_t             &V,
              const DeformerType &typeDf,
              const BulgingType  &typeGamma,
              const double       sigma,
              const double       blend);

  matd_t barycenters() const;

  // io
  int save_cage(const char *file) const;
  int save_register_cage(const char *file) const;
  int save_mesh(const char *file) const;  
  int save_bases(const char *file, DeformerType dt) const;

 private:
  void calc_outward_normal();
  void calc_vert_normal();
  void calc_double_area();
  
 public:
  double mu_, nu_;
  double runtime;

  // mesh
  Eigen::MatrixXi F_;
  Eigen::Matrix3Xd V_;

  // cage
  Eigen::MatrixXi cageF_;
  Eigen::Matrix3Xd cageV_, cageN_, cageNV_;
  vecd_t cageA_;

  // cage in CGAL
  Surface_mesh cage_;
  std::map<Surface_mesh::Vertex_index, size_t> v2i_;
  std::map<Surface_mesh::Face_index, size_t> f2i_;

  // registered cage
  Eigen::Matrix3Xd cageVr_;

  // initial states
  Eigen::Matrix3Xd cageV0_, cageN0_, cageNV0_, V0_;
  vecd_t cageA0_;
  double volumeV0_;
  std::vector<mat3d> Frm0_;
  std::vector<Eigen::Matrix2d> Dm0_;

  // rotations of facets, nodes, and query points
  std::vector<mat3d> Rf_, Rv_;

  // Green coordinates
  matd_t phi_, psi_;

  // meavalue
  matd_t Phi_;

  // Somigliana coordinates
  matd_t PHI_, PSI_;
  vecd_t initSA_;
  vecd_t sf_;
  Eigen::Matrix3Xd dudn_;

  // phong embedding
  mati_t tetF_;
  matd_t tetV0_;
  matd_t tetBC0_;
  Eigen::SparseMatrix<double> phong_;
  std::vector<std::vector<std::pair<size_t, double>>> tetV0_wk_;
  std::shared_ptr<tet_fem> tet_fem_;
  Eigen::SparseMatrix<double> tetS_;

  // GL integrator
  std::shared_ptr<trig_integrator> trig_it_;

  // cuda precomputer
  std::shared_ptr<cage_precomputer> cage_prec;
};

}
#endif