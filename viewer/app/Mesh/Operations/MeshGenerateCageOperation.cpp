#include <Mesh/Operations/MeshGenerateCageOperation.h>
#include <Mesh/Operations/CageGenerationSteps/Voxelizer.h>
#include <Mesh/Operations/CageGenerationSteps/ClosingOperator.h>
#include <Mesh/Operations/CageGenerationSteps/RemeshOperator.h>
#include <iostream>
#include <utility>
#include <array>
#include <fstream>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <ctime>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <boost/optional/optional_io.hpp>

typedef Exact_Kernel::Point_3										ExactPoint;
typedef Inexact_Kernel::Point_3										InexactPoint;
typedef CGAL::Surface_mesh<InexactPoint>							Mesh;
typedef CGAL::Surface_mesh<ExactPoint>								ExactMesh;
typedef std::vector<bool> VOXEL_GRID;
typedef std::vector<VOXEL_GRID>	MIPMAP_TYPE;

// save diagnostic state
#pragma GCC diagnostic push 

// turn off the specific warning. Can also use "-Wall"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmissing-template-arg-list-after-template-kw"


float cell_size;
std::array<float, 3> bbox_min;

void GenerateCageFromMeshOperation::Execute() {
	
	int cage_start = clock();

	std::string filename = _params._meshfilepath.string();
	std::string outputfilename = _params._cagefilepath.string();

	//extract input model name
	std::string obj = filename.substr(filename.find_last_of('\\') + 1, filename.find_last_of('.') - 1);
	std::string filepath = filename.substr(0, filename.find_last_of('\\') + 1);
	std::string intermediate_path = filepath + obj + "_interm.obj";
	
	VOXEL_GRID& e_grid = _params._closingResult;
	int resolution = pow(2, _params._voxelResolution);
	std::cout << "Generating Cage for " << obj << ", resol: " << resolution << std::endl;

	if (e_grid.size() != std::pow(resolution, 3))
	{	
		float se_scale = resolution / 16.f;
		Voxelizer voxelizer(resolution, se_scale);
		VOXEL_GRID voxel_result = voxelizer.GenerateVoxelGrid(filename);
		
		cell_size = voxelizer.GetCellSize();
		bbox_min = voxelizer.GetBboxMin();

		ClosingOperator closer(resolution, se_scale, cell_size, bbox_min, voxel_result);
		closer.ExecuteDilation();
		closer.ExtractContour();
		VOXEL_GRID erosion_result = closer.ExecuteErosion();
		e_grid.resize(erosion_result.size());
		e_grid.assign(erosion_result.begin(), erosion_result.end());
	}

	RemeshOperator remesher(resolution, cell_size);
	ExactMesh extracted_surface = remesher.ExtractSurface(e_grid, bbox_min);
	CGAL::write_off(intermediate_path.c_str(), extracted_surface);

	// Simplification
	MyMesh final_mesh;
	tri::io::ImporterOFF<MyMesh>::Open(final_mesh, intermediate_path.c_str());
	remesher.Decimate(final_mesh, _params._smoothIterations, _params._targetNumFaces);
	
	tri::io::ExporterOBJ<MyMesh>::Save(final_mesh, outputfilename.c_str(), tri::io::Mask::IOM_BITPOLYGONAL);

	int cage_end = clock();
	printf("[Cage Generation Done] elapsed time: %5.3f sec\n", float(cage_end - cage_start) / CLOCKS_PER_SEC);
}
