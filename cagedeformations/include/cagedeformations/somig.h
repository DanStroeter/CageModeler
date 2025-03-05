#ifndef SOMIGLIANA_3D_H
#define SOMIGLIANA_3D_H

#include <vector>
#include <Eigen/Dense>
#include "point3.h"

typedef Eigen::Matrix<size_t, -1, -1> mati_t;
typedef Eigen::Matrix<double, -1, -1> matd_t;
typedef Eigen::Matrix<double, -1,  1> vecd_t;
typedef Eigen::Matrix3d mat3d;

class somig_deformer_3
{
 public:
  typedef std::vector< std::vector< unsigned int > > face_t;
  typedef std::vector< point3d > vert_t;
  
  somig_deformer_3() : mu_(1.0), nu_(0.0) {}
  somig_deformer_3(const double nu) : mu_(1.0), nu_(nu) {}

  void set_mesh(const face_t &F, const vert_t &V);
  void set_cage(const face_t &F, const vert_t &V);  

  size_t num_cage_vertices() const { return cageV_.cols(); }
  size_t num_mesh_vertices() const { return V_.cols(); }
  size_t num_cage_facets() const { return cageF_.cols(); }

  void precompute_somig_coords(const bool sanity_check);
  void deform(const vert_t &cageV, vert_t &meshV);
  
  void calc_outward_normal();

  matd_t& getPhi() { return PHI_; };
  
 private:
  double mu_, nu_;

  // mesh
  mati_t F_;
  matd_t V_;

  // cage
  mati_t cageF_;
  matd_t cageV_, cageN_;

  // initial
  matd_t cageV0_, cageN0_, V0_;
  
  // Somigliana coordinates
  matd_t PHI_, PSI_;
  vecd_t sf_;
  matd_t dudn_;
};

#endif
