#include <somigliana/io.h>
#include <numeric>
#include <somigliana/macro.h>
#include <somigliana/vtk.h>

using namespace std;

namespace green {

int tet_mesh_read_from_vtk(const char *path, matd_t &nods, mati_t &tets) {
  ifstream ifs(path);
  if(ifs.fail()) {
    cerr << "[info] cannot open file " << path << endl;
    return __LINE__;
  }

  string str;
  int point_num = 0, cell_num = 0;

  while(!ifs.eof()){
    ifs >> str;
    if(str == "POINTS"){
      ifs >> point_num >> str;
      nods.resize(3, point_num);
      for(size_t i = 0;i < point_num; ++i){
        for(size_t j = 0;j < 3; ++j) {
          ifs >> nods(j, i);
        }
      }
      continue;
    }
    
    if(str == "CELLS"){
      ifs >> cell_num >> str;
      int point_number_of_cell = 0;
      vector<size_t> tet_temp;
      for(size_t ci = 0; ci < cell_num; ++ci){
        ifs >> point_number_of_cell;
        if(point_number_of_cell != 4){
          for(size_t i = 0; i < point_number_of_cell; ++i)
            ifs >> str;
        }else{
          int p;
          for(size_t i = 0; i < point_number_of_cell; ++i){
            ifs >> p;
            tet_temp.push_back(p);
          }
        }
      }
      tets.resize(4, tet_temp.size()/4);
      std::copy(tet_temp.begin(), tet_temp.end(), tets.data());
    }
  }
  
  return 0;
}

int hex_mesh_write_to_vtk(const char *path, const matd_t &nods, const mati_t &hexs,
                          const vecd_t *mtr, const char *type) {
  ASSERT(hexs.rows() == 8);
  
  ofstream ofs(path);
  if ( ofs.fail() ) {
    return __LINE__;
  }
  
  hex2vtk(ofs, nods.data(), nods.size()/3, hexs.data(), hexs.size()/8);

  if ( mtr != nullptr ) {
    const string mtr_name = "theta";
    ofs << type << "_DATA " << mtr->size() << "\n";
    vtk_data(ofs, mtr->data(), mtr->size(), mtr_name.c_str(), mtr_name.c_str());
  }
  ofs.close();

  return 0;
}

int point_write_to_vtk(const char *path, const matd_t &nods) {
  ofstream ofs(path);
  if ( ofs.fail() )
    return __LINE__;

  matd_t nods_to_write = matd_t::Zero(3, nods.cols());
  if ( nods.rows() == 2 ) {
    nods_to_write.topRows(2) = nods;
  } else if ( nods.rows() == 3 ) {
    nods_to_write = nods;
  }

  std::vector<size_t> cell(nods.cols());
  std::iota(cell.begin(), cell.end(), 0);
  point2vtk(ofs, nods_to_write.data(), nods_to_write.cols(), &cell[0], cell.size());
  ofs.close();
        
  return 0;
}

int tri_mesh_write_to_vtk(const char *path, const matd_t &nods, const mati_t &tris,
                          const vecd_t *mtr, const char *type) {
  ASSERT(tris.rows() == 3);

  ofstream ofs(path);
  if ( ofs.fail() ) {
    return __LINE__;
  }
  
  tri2vtk(ofs, nods.data(), nods.size()/3, tris.data(), tris.size()/3);

  if ( mtr != nullptr ) {
    const string mtr_name = "theta";
    ofs << type << "_DATA " << mtr->size() << "\n";
    vtk_data(ofs, mtr->data(), mtr->size(), mtr_name.c_str(), mtr_name.c_str());
  }
  ofs.close();

  return 0;  
}

int tri_mesh_write_to_vtk(const char *path, const matd_t &nods, const mati_t &tris,
                          const matd_t *mtr, const char *type) {
  ASSERT(tris.rows() == 3);

  ofstream ofs(path);
  if ( ofs.fail() ) {
    return __LINE__;
  }
  
  tri2vtk(ofs, nods.data(), nods.size()/3, tris.data(), tris.size()/3);

  if ( mtr != nullptr ) {
    for (int i = 0; i < mtr->rows(); ++i) { // for each field
      const string mtr_name = "field"+to_string(i);
      vecd_t curr_field = mtr->row(i);
      std::for_each(curr_field.data(), curr_field.data()+curr_field.size(),
                    [](double &x) { x = fabs(x) < 1e-12 ? 0 : x; });
      if ( i == 0 ) {
        ofs << type << "_DATA " << curr_field.size() << "\n";
      }
      vtk_data(ofs, curr_field.data(), curr_field.size(), mtr_name.c_str(), mtr_name.c_str());
    }
  }
  ofs.close();

  return 0;  
}

int save_wavefront_obj(const char *filename, const mati_t &cell, const matd_t &nods) {
  std::ofstream file(filename);
  if ( file.fail() ) {
    cerr << "# cannot open " << filename << endl;
    return __LINE__;
  }

  for(int i = 0; i < nods.cols(); ++i) {
    file << std::fixed;
    file << "v " << nods(0, i) << " " << nods(1, i) << " " << nods(2, i) << std::endl;
  }

  for (int i = 0; i < cell.cols(); ++i) {
    file << "f ";
    for(int j = 0; j < cell.rows(); ++j) {
      file << cell(j, i) + 1 << " ";
    }
    file << std::endl;
  }
  
  return 0;
}

}
