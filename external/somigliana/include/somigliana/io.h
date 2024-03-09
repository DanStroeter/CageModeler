#ifndef IO_H
#define IO_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <iterator>
#include <Eigen/Sparse>
#include <somigliana/types.h>

namespace green {

int tet_mesh_read_from_vtk(const char *path, matd_t &nods, mati_t &tets);

int hex_mesh_write_to_vtk(const char *path, const matd_t &nods, const mati_t &hexs,
                          const vecd_t *mtr=nullptr, const char *type="CELL");

int point_write_to_vtk(const char *path, const matd_t &nods);

int tri_mesh_write_to_vtk(const char *path, const matd_t &nods, const mati_t &tris,
                          const vecd_t *mtr=nullptr, const char *type="CELL");

int tri_mesh_write_to_vtk(const char *path, const matd_t &nods, const mati_t &tris,
                          const matd_t *mtr, const char *type);

int save_wavefront_obj(const char *filename, const mati_t &cell, const matd_t &nods);

template <class T>
void print_vector(const std::vector<T> &input) {
  std::copy(input.begin(), input.end(), std::ostream_iterator<T>(std::cout, " "));
  std::cout << std::endl;
}

template <class Mat>
struct io_mat_trait {
  typedef typename Mat::value_type value_type;
};

template <typename T>
struct io_mat_trait<Eigen::Matrix<T, -1, -1>> {
  typedef T value_type;
};
  
template <typename T>
struct io_mat_trait<Eigen::Matrix<T, -1, 1>> {
  typedef T value_type;
};

template <class Mat>
int read_dense_matrix(const char *path, Mat &dat) {
  typedef typename io_mat_trait<Mat>::value_type T;
  std::ifstream ifs(path, std::ios::binary);
  if ( ifs.fail() )
    return __LINE__;
  int64_t mat_size[2];
  ifs.read((char *)&mat_size[0], 2*sizeof(int64_t));
  dat.resize(mat_size[0], mat_size[1]);
  ifs.read((char *)dat.data(), dat.size()*sizeof(T));
  ifs.close();
  return 0;
}

template <class Mat>
int write_dense_matrix(const char *path, const Mat &dat) {
  typedef typename io_mat_trait<Mat>::value_type T;
  std::ofstream ofs(path, std::ios::binary);
  if ( ofs.fail() )
    return __LINE__;
  const int64_t mat_size[2] = {dat.rows(), dat.cols()};
  ofs.write((const char *)&mat_size[0], 2*sizeof(int64_t));
  ofs.write((const char *)dat.data(), dat.size()*sizeof(T));
  ofs.close();
  return 0;
}

template <typename T>
int read_sparse_matrix(const char *path, Eigen::SparseMatrix<T> &mat) {
  std::ifstream ifs(path, std::ios::binary);
  if ( ifs.fail() )
    return __LINE__;

  Eigen::Index rows, cols, nnz;
  ifs.read((char *)&rows, sizeof(Eigen::Index));
  ifs.read((char *)&cols, sizeof(Eigen::Index));
  ifs.read((char *)&nnz,  sizeof(Eigen::Index));

  std::vector<int> ptr(cols+1), idx(nnz);
  std::vector<T> val(nnz);
  ifs.read((char *)&ptr[0], ptr.size()*sizeof(int));
  ifs.read((char *)&idx[0], idx.size()*sizeof(int));
  ifs.read((char *)&val[0], val.size()*sizeof(T));
  ifs.close();
  
  mat = Eigen::Map<Eigen::SparseMatrix<T>>(rows, cols, nnz, &ptr[0], &idx[0], &val[0]);

  return 0;
}

template <typename T>
int write_sparse_matrix(const char *path, const Eigen::SparseMatrix<T> &mat) {
  //-> only for col major sparse matrix
  Eigen::SparseMatrix<T> mat_tm = mat;
  if ( !mat_tm.isCompressed() )
    mat_tm.makeCompressed();

  std::ofstream ofs(path, std::ios::binary);
  if ( ofs.fail() )
    return __LINE__;
  
  const Eigen::Index rows = mat_tm.rows(), cols = mat_tm.cols(), nnz = mat_tm.nonZeros();
  ofs.write((const char *)&rows, sizeof(Eigen::Index));
  ofs.write((const char *)&cols, sizeof(Eigen::Index));
  ofs.write((const char *)&nnz,  sizeof(Eigen::Index));
  //-> the default SparseMatrix::StorageIndex is int
  ofs.write((const char *)mat_tm.outerIndexPtr(), (cols+1)*sizeof(int));
  ofs.write((const char *)mat_tm.innerIndexPtr(), nnz*sizeof(int));
  ofs.write((const char *)mat_tm.valuePtr(),      nnz*sizeof(T));
  ofs.close();

  return 0;
}

template <typename T>
int readXY(const char *filename, std::vector<T> &X, std::vector<T> &Y) {
  std::ifstream ifs(filename);
  if ( ifs.fail() ) {
    std::cerr << "# cannot open " << filename << std::endl;
    return __LINE__;
  }
  if ( !X.empty() ) X.clear();
  if ( !Y.empty() ) Y.clear();

  T x, y;
  while ( ifs >> x >> y ) {
    X.emplace_back(x);
    Y.emplace_back(y);    
  }
  ifs.close();  
  return 0;
}

}
#endif
