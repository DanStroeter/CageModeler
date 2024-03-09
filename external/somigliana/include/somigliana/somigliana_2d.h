#ifndef SOMIGLIANA_COO_H
#define SOMIGLIANA_COO_H

#include <Eigen/Sparse>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <map>
#include "types.h"

namespace green {

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3  Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;

typedef Eigen::Matrix2d mat2d;
typedef Eigen::Vector2d vec2d;

struct plot_info
{
  int eid_;
};

class somigliana_deformer2
{
 public:
  somigliana_deformer2()
      : mu_(1.0), nu_(0.0) {}
  somigliana_deformer2(const double nu)
      : mu_(1.0), nu_(nu) {}
  
  void set_nu(const double nu) {
    nu_ = nu;
  }
  double get_nu() const { return nu_; }
  
  int load_mesh(const std::string &file);
  int load_cage(const std::string &file);
  void init(const size_t num_quadrature);

  size_t num_cage_vertices() const { return cageV_.cols(); }
  size_t num_mesh_vertices() const { return V_.cols(); }
  size_t num_cage_edges() const { return cageE_.cols(); }

  // precompute three types of coordinates
  void precompute_meanv_coords();
  void precompute_somig_coords();
  void precompute_green_coords();

  // return deformed vertices and corotated frames (dx, dy)^T
  void deform(matd_t             &V,
              const DeformerType &typeDf,
              const BulgingType  &typeGamma,
              const double       sigma,
              const double       blend,
              const plot_info    &plt,              
              matd_t             &dx,
              matd_t             &dy,
              matd_t             &bc,
              matd_t             &trac);

  int save_mesh(const std::string &file);
  int save_normal(const std::string &file);
  int save_cage(const std::string &file);
  int save_register_cage(const std::string &file);

  double mesh_area() const;

  void clean_angles() { prevTA_ = initTA_; }

 private:
  void calc_outward_normal();
  void calc_edge_length();
  void calc_cage_grad(const char df_type);
  void calc_vert_normal();

 public:
  // raw formats using CGAL
  Surface_mesh cage_, mesh_;
  std::map<Surface_mesh::Vertex_index, size_t> v2i_;

  double mu_, nu_;

  // mesh 
  Eigen::MatrixXi F_;
  matd_t V_, V0_;

  // cage
  Eigen::MatrixXi cageE_, cageF_;
  matd_t cageV_, cageN_, cageVN_;
  vecd_t cageL_;
  std::vector<size_t> cageV_loop_;

  // register rest cage to the deformed one
  matd_t cageVr_;
  
  // initial states
  matd_t cageV0_, cageN0_, cageVN0_;
  vecd_t cageL0_;
  double areaA0_;

  // edge-based, cage vertex based and mesh rotation
  std::vector<mat2d> Re_, Rv_;

  // mean-value coordinates
  matd_t Phi_;
  
  // Green coordinates
  matd_t phi_, psi_; // psiPoU_;

  // Somigliana coordinates
  matd_t PHI_, PSI_;
  vecd_t prevTA_, initTA_;
  Eigen::Matrix2Xd dudn_;
  vecd_t se_;

  // quadratures
  std::vector<double> qp_, qw_;
};

}
#endif
