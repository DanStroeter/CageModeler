#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/CageGenerationSteps/Utils.h>
#include <cagedeformations/LoadMesh.h>
#include <cagedeformations/GreenCoordinates.h>
#include <filesystem>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <omp.h>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
//#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>

#include <CGAL/Polygon_mesh_processing/intersection.h>
//#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
//#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
//#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
//#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
//#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
//#include <CGAL/Polygon_mesh_processing/compute_normal.h>
//#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>

#define BUFFER_OFFSET(offset) ((GLvoid*)(offset))

typedef CGAL::Exact_predicates_exact_constructions_kernel			Exact_Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel			Inexact_Kernel;
typedef CGAL::Polyhedron_3<Inexact_Kernel>							Inexact_Polyhedron;
typedef CGAL::Polyhedron_3<Exact_Kernel>							Exact_Polyhedron;
typedef Exact_Kernel::Point_3										ExactPoint;
typedef Inexact_Kernel::Point_3										InexactPoint;
typedef Exact_Kernel::Vector_3										ExactVector;
typedef CGAL::Surface_mesh<InexactPoint>							Mesh;
typedef CGAL::AABB_face_graph_triangle_primitive<Exact_Polyhedron>	Primitive;

typedef CGAL::AABB_traits<Exact_Kernel, Primitive>					Traits;
typedef CGAL::AABB_tree<Traits>										Tree;
typedef CGAL::Side_of_triangle_mesh<Exact_Polyhedron, Exact_Kernel>	Point_inside;
typedef std::vector<bool> VOXEL_GRID;
typedef std::vector<VOXEL_GRID>	MIPMAP_TYPE;

namespace PMP = CGAL::Polygon_mesh_processing;

class Voxelizer: public Utils {

public:
	Voxelizer(int resolution, float se_size) : _resolution(resolution), _seSize(se_size) {
		_window = initOpenGL();
	}
	~Voxelizer() {};

private:
	GLFWwindow* _window;
	GLuint _programVolume;
	GLuint _programSurface;

	GLuint _vao = 0;
	GLuint _vertexVboId = 0;
	GLuint _normalVboId = 0;
	GLuint _indexVboId = 0;

	GLuint _voxFbo;
	GLuint _voxFboTex;
	GLuint _voxTex;

	int _resolution = 32;
	float _seSize = _resolution / 16;
	float _cellSize = 0.f;


	std::vector<float> _vertices;
	std::vector<float> _normals;
	std::vector<unsigned int> _indices;
	VOXEL_GRID _voxelGrid;

	std::array<float, 3> _bboxMin = { FLT_MAX, FLT_MAX, FLT_MAX };
	std::array<float, 3> _bboxMax = { FLT_MIN, FLT_MIN, FLT_MIN };

	std::array<float, 3> _gridBboxMin = { FLT_MAX, FLT_MAX, FLT_MAX };
	std::array<float, 3> _gridBboxMax = { FLT_MIN, FLT_MIN, FLT_MIN };

private:
	GLuint LoadShader(const std::string& path, GLenum shaderType) {
		std::ifstream shaderFile(path);
		if (!shaderFile.is_open()) {
			std::cerr << "[ERROR] Failed to open shader file: " << path << std::endl;
			return 0;
		}

		std::stringstream shaderStream;
		shaderStream << shaderFile.rdbuf();
		std::string shaderCode = shaderStream.str();
		shaderFile.close();

		const char* shaderSource = shaderCode.c_str();
		GLuint shader = glCreateShader(shaderType);
		glShaderSource(shader, 1, &shaderSource, NULL);
		glCompileShader(shader);

		// Check Compile Error
		GLint success;
		glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
		if (!success) {

			char infoLog[512];
			glGetShaderInfoLog(shader, 512, NULL, infoLog);
			std::cerr << "[ERROR] Shader Compilation Failed: " << path << "\n" << infoLog << std::endl;
			return 0;
		}

		return shader;
	}
	GLuint GenVGFProgram(const std::string& vertexPath,
		const std::string& geometryPath,
		const std::string& fragmentPath) {
		GLuint vertexShader = LoadShader(vertexPath, GL_VERTEX_SHADER);
		GLuint geometryShader = LoadShader(geometryPath, GL_GEOMETRY_SHADER);
		GLuint fragmentShader = LoadShader(fragmentPath, GL_FRAGMENT_SHADER);

		if (vertexShader == 0 || geometryShader == 0 || fragmentShader == 0) {
			std::cerr << "[ERROR] Shader loading failed. Program creation aborted." << std::endl;
			return 0;
		}

		GLuint program = glCreateProgram();
		glAttachShader(program, vertexShader);
		glAttachShader(program, geometryShader);
		glAttachShader(program, fragmentShader);

		glLinkProgram(program);

		GLint success;
		glGetProgramiv(program, GL_LINK_STATUS, &success);
		if (!success) {
			char infoLog[512];
			glGetProgramInfoLog(program, 512, NULL, infoLog);
			std::cerr << "[ERROR] Shader Program Linking Failed\n" << infoLog << std::endl;
			return 0;
		}

		glDeleteShader(vertexShader);
		glDeleteShader(geometryShader);
		glDeleteShader(fragmentShader);

		return program;
	}

	GLFWwindow* initOpenGL() {

		if (!glfwInit()) {
			std::cerr << "Failed to initialize GLFW" << std::endl;
			return nullptr;
		}

		// Set OpenGL context version (OpenGL 4.5)
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

		// Create window
		GLFWwindow* window = glfwCreateWindow(1, 1, "OpenGL 4.5 - GLSL 450", NULL, NULL);
		if (!window) {
			std::cerr << "Failed to create GLFW window" << std::endl;
			glfwTerminate();
			return nullptr;
		}

		// Bind OpenGL to the current thread
		glfwMakeContextCurrent(window);

		// GLEW Initialization
		glewExperimental = GL_TRUE;
		GLenum err = glewInit();
		if (err != GLEW_OK) {
			std::cerr << "GLEW init failed: " << glewGetErrorString(err) << std::endl;
			return nullptr;
		}

		std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
		std::cout << "GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

		return window;
	}

	void buildFBO() {
		// func: buildFBO

		int bucket_size = 32;
		int num_bit_vox = 1;
		int res_x = 2 * _resolution, res_y = 2 * _resolution, res_z = 2 * _resolution / ((bucket_size / num_bit_vox));

		// Some general OpenGL setup :	
		// Render Target (FBO + attached texture), viewport, depth test etc.

		// Binding to a dummy fbo
		glGenFramebuffers(1, &_voxFbo);
		glBindFramebuffer(GL_FRAMEBUFFER, _voxFbo);

		// Binding dummy texture
		glEnable(GL_TEXTURE_2D);
		glGenTextures(1, &_voxFboTex);
		glBindTexture(GL_TEXTURE_2D, _voxFboTex);

		// Setting dummy texture Tex Parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		// Allocating dummy texture
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, res_x, res_y, 0,
			GL_RGBA, GL_FLOAT, NULL);

		// Attaching dummy texture to dummy fbo
		glFramebufferTexture2D(GL_FRAMEBUFFER,
			GL_COLOR_ATTACHMENT0,
			GL_TEXTURE_2D,
			_voxFboTex, 0);

		// Setting viewport
		glViewport(0, 0, res_x, res_y);

		// Disabling Depth Test
		glDisable(GL_DEPTH_TEST);

		// Disabling Framebuffer writing
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

		// Disabling Face Culling
		glDisable(GL_CULL_FACE);

		// Create the texture we're going to render to (Voxel Texture)
		glEnable(GL_TEXTURE_3D);
		glGenTextures(1, &_voxTex);

		// Bind Voxel Texrue
		glBindTexture(GL_TEXTURE_3D, _voxTex);

		// Give an empty image to OpenGL ( the last "0" )
		// Allocating Voxel Texture
		int tex_level = 0;
		int border = 0;
		glTexImage3D(GL_TEXTURE_3D, tex_level, GL_R32UI, res_x, res_y, res_z,
			border, GL_RED_INTEGER, GL_UNSIGNED_INT, 0);

		// Setting Tex Parameter
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);


		// Clear the texture
		unsigned int vox_tex_clear_value = 0;
		glBindTexture(GL_TEXTURE_3D, _voxTex);

		glClearTexImage(_voxTex, tex_level, GL_RED_INTEGER, GL_UNSIGNED_INT,
			&vox_tex_clear_value);

		glMemoryBarrier(GL_TEXTURE_UPDATE_BARRIER_BIT);

		// Bind the texture to an image unit
		glBindImageTexture(0, _voxTex, tex_level,
			GL_TRUE,
			0,
			GL_READ_WRITE,
			GL_R32UI
		);
	}

	void RunShader(GLuint program, bool is_surface, float rescale, int res, float sq_ar_thresh) {

		GLuint query;
		GLuint64 elapsed_time;
		glGenQueries(1, &query);

		// 1) Use the Volume Voxelization Shader
		glUseProgram(program);

		// 2) Initialization of the shader parameters
		// Vertex and Fragment Shader Common Setup
		glUniform3f(glGetUniformLocation(program, "bbox_min"),
			_bboxMin[0], _bboxMin[1], _bboxMin[2]);
		glUniform1f(glGetUniformLocation(program, "rescale"), rescale);
		glUniform1ui(glGetUniformLocation(program, "res"), res);

		if (is_surface) {
			// Geometry Shader Setup
			glUniform1f(glGetUniformLocation(program, "sq_ar_thresh"), sq_ar_thresh);
		}

		// Fragment Shader Setup : assign image unit
		glUniform1i(glGetUniformLocation(program, "vox_grid"), 0);

		// 3) Single Pass  Voxelization

		if (glewIsSupported("GL_NV_conservative_raster")) {
			if (is_surface) {
				glEnable(GL_CONSERVATIVE_RASTERIZATION_NV);
				//std::cout << "enabling GL_CONSERVATIVE_RASTERIZATION_NV\n";
			}
			else {
				glDisable(GL_CONSERVATIVE_RASTERIZATION_NV);
				//std::cout << "disabling GL_CONSERVATIVE_RASTERIZATION_NV\n";
			}
		}
		else if (glewIsSupported("GL_INTEL_conservative_rasterization")) {
			if (is_surface) {
				glEnable(GL_CONSERVATIVE_RASTERIZATION_INTEL);
				//std::cout << "enabling GL_CONSERVATIVE_RASTERIZATION_INTEL\n";
			}
			else {
				glDisable(GL_CONSERVATIVE_RASTERIZATION_INTEL);
				//std::cout << "disabling GL_CONSERVATIVE_RASTERIZATION_INTEL\n";
			}

		}
		else {
			//std::cerr << "[WARNING] GL_NV_conservative_raster/GL_INTEL_conservative_rasterization is not supported on this GPU." << std::endl;
		}

		glBeginQuery(GL_TIME_ELAPSED, query);

		glBindVertexArray(_vao);
		glDrawElements(GL_TRIANGLES, _indices.size(), GL_UNSIGNED_INT, BUFFER_OFFSET(0));
		glBindVertexArray(0);

		glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		glEndQuery(GL_TIME_ELAPSED);

		// Retrieving the recorded elapsed time : wait until
		// the query result is available
		int done = 0;
		while (!done)
			glGetQueryObjectiv(query, GL_QUERY_RESULT_AVAILABLE, &done);

		// Get the query result for elapsed time
		glGetQueryObjectui64v(query, GL_QUERY_RESULT, &elapsed_time);

		/*if (is_surface) {
			std::cout << "[Voxelizer] : " << "conservative surface voxelization done in "
				<< elapsed_time / 1000000.0 << " ms." << std::endl;
		}
		else {
			std::cout << "[Voxelizer] : " << "conservative volume voxelization done in "
				<< elapsed_time / 1000000.0 << " ms." << std::endl;
		}*/
	}

	void ClearBuffer() {

		// Delete the dummy fbo and its attached texture
		glDeleteTextures(1, &_voxFboTex);
		glDeleteFramebuffers(1, &_voxFbo);

		// 4) Restore a "standard" OpenGL state
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glEnable(GL_CULL_FACE);
		glDisable(GL_TEXTURE_3D);
		glEnable(GL_DEPTH_TEST);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		// Delete VBO and VAO
		glDeleteBuffers(1, &_vertexVboId);
		glDeleteBuffers(1, &_normalVboId);
		glDeleteBuffers(1, &_indexVboId);
		glDeleteVertexArrays(1, &_vao);
	}

	void ConvertBucketToUnpackedGrid(const std::vector<unsigned int>& buck_vox_grid,
		int res, int num_bit_vox,
		VOXEL_GRID& unpack_vox_grid) {
		// Unpacked grid의 크기는 2x2x2 범위로 하나의 voxel이 정의된 크기만큼 나눔
		int unpack_res_x = res / 2, unpack_res_y = res / 2, unpack_res_z = res / 2;
		unpack_vox_grid.resize(unpack_res_x * unpack_res_y * unpack_res_z, false);

		int res_x = res, res_y = res, res_z = res / ((32 / num_bit_vox));

		// 2x2x2 block to one voxel
		for (int i = 0; i < res_x; i++) {
			for (int j = 0; j < res_y; j++) {
				for (int k = 0; k < res_z; k++) {
					unsigned int value = buck_vox_grid[i + res_x * j + res_x * res_y * k];

					if (value != 0) {

						for (int s = 0; s < 32; s++) {
							if ((value & (0x00000001)) == 0x00000001) {

								int pos[3] = { i, j, k * 32 + s };
								int target_pos[3] = { pos[0] / 2, pos[1] / 2, pos[2] / 2 };

								int target_pos_flat = target_pos[2] + target_pos[1] * unpack_res_z + target_pos[0] * unpack_res_z * unpack_res_y;
								unpack_vox_grid[target_pos_flat] = true;
							}
							value /= 2;
						}
					}
				}
			}
		}
	}

	void ReadVoxelGrid() {

		int bucket_size = 32;
		int num_bit_vox = 1;
		int res_x = 2 * _resolution, res_y = 2 * _resolution, res_z = 2 * _resolution / ((bucket_size / num_bit_vox));

		std::vector<unsigned int> buck_vox_grid(res_x * res_y * res_z, 0);
		glBindTexture(GL_TEXTURE_3D, _voxTex);

		int get_tex_level = 0;
		glGetTexImage(GL_TEXTURE_3D, get_tex_level,
			GL_RED_INTEGER,
			GL_UNSIGNED_INT,
			&buck_vox_grid[0]);

		ConvertBucketToUnpackedGrid(buck_vox_grid, _resolution * 2, num_bit_vox, _voxelGrid);
	}

	void SetUpVAO() {
		// 1) init vbo
		_vao = 0;

		glGenVertexArrays(1, &_vao);
		glBindVertexArray(_vao);

		// Create vertex vbo and write data
		glGenBuffers(1, &_vertexVboId);
		glBindBuffer(GL_ARRAY_BUFFER, _vertexVboId);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * _vertices.size(), _vertices.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
		glEnableVertexAttribArray(0);

		// Create normal vbo and write data
		glGenBuffers(1, &_normalVboId);
		glBindBuffer(GL_ARRAY_BUFFER, _normalVboId);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * _normals.size(), _normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
		glEnableVertexAttribArray(1);

		// Create EBO (Index Buffer Object) and write data
		glGenBuffers(1, &_indexVboId);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _indexVboId);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, _indices.size() * sizeof(unsigned int), _indices.data(), GL_STATIC_DRAW);

		// Unbind VAO
		glBindVertexArray(0);
	}

	VOXEL_GRID VoxelizeShader(std::string filename) {

		if (!_window) return _voxelGrid; // return empty vector

		Eigen::MatrixXd V_model, N_model;
		Eigen::MatrixXi T_model;

		if (!load_mesh(filename, V_model, T_model, 1.0))
		{
			VOXEL_GRID dm;
			std::cerr << "Failed to load mesh file\n";
			return dm;
		}

		calcNormals(V_model, T_model, N_model);

		// Convert Eigen::MatrixXd (vertices) to std::vector<float>
		for (int i = 0; i < V_model.rows(); ++i) {
			for (int j = 0; j < V_model.cols(); ++j) {
				_vertices.push_back(static_cast<float>(V_model(i, j)));

				if (V_model(i, j) < _bboxMin[j]) _bboxMin[j] = static_cast<float>(V_model(i, j));
				else if (V_model(i, j) > _bboxMax[j]) _bboxMax[j] = static_cast<float>(V_model(i, j));
			}
		}

		for (int i = 0; i < N_model.rows(); ++i) {
			for (int j = 0; j < N_model.cols(); ++j) {
				_normals.push_back(static_cast<float>(N_model(i, j)));
			}
		}

		for (int i = 0; i < T_model.rows(); ++i) {
			for (int j = 0; j < T_model.cols(); ++j) {
				_indices.push_back(static_cast<unsigned int>(T_model(i, j)));
			}
		}

		// 0) Set the cell size

		float longest_axis = std::max(_bboxMax[0] - _bboxMin[0],
			std::max(_bboxMax[1] - _bboxMin[1], _bboxMax[2] - _bboxMin[2]));

		float margin = 0.1f + 2.0f / (2 * _resolution);
		float thickness = longest_axis / _resolution;
		float offset = 2 * thickness + margin;
		float axis_len = longest_axis + 2 * offset;
		float cell_size = axis_len / _resolution;
		_cellSize = cell_size;

		_bboxMin[0] -= offset; _bboxMin[1] -= offset; _bboxMin[2] -= offset;

		// check if this var is needed
		_bboxMax[0] = _bboxMin[0] + axis_len;
		_bboxMax[1] = _bboxMin[1] + axis_len;
		_bboxMax[2] = _bboxMin[2] + axis_len;

		SetUpVAO();

		int base_res_ = _resolution;

		// shader creates (_resolution * 2)^3 grid then we extract max out of 2x2x2 block
		int res = 2 * base_res_;
		int num_bit_vox = 1; // number of bits in one bucket
		float rescale = 2.0 / axis_len;

		// Load shader programs
		_programVolume = GenVGFProgram(
			"assets/shaders/voxelizer/vol_vox.vert",
			"assets/shaders/voxelizer/vol_vox.geo",
			"assets/shaders/voxelizer/vol_vox_bit.frag");

		_programSurface = GenVGFProgram(
			"assets/shaders/voxelizer/surf_vox_conserv.vert",
			"assets/shaders/voxelizer/surf_vox_conserv.geo",
			"assets/shaders/voxelizer/surf_vox_conserv_bit.frag");

		buildFBO();

		// volume voxelization
		RunShader(_programVolume, false, rescale, res, 0);

		// surface voxelization
		RunShader(_programSurface, true, rescale, res, 50.f * 50.f);

		ClearBuffer();
		ReadVoxelGrid();

		glfwDestroyWindow(_window);
		glfwTerminate();

		return _voxelGrid;
	}

	VOXEL_GRID VoxelizeCGAL(std::string filename)
	{

		Exact_Polyhedron poly;
		if (!PMP::IO::read_polygon_mesh(filename, poly) || !CGAL::is_triangle_mesh(poly))
		{
			std::cerr << "Invalid input.\n";
			exit(-1);
		}

		Tree tree(faces(poly).first, faces(poly).second, poly);

		CGAL::Bbox_3 bbox_origin = tree.bbox();
		float longest_axis = std::max(bbox_origin.xmax() - bbox_origin.xmin(),
			std::max(bbox_origin.ymax() - bbox_origin.ymin(), bbox_origin.zmax() - bbox_origin.zmin()));
		
		float margin = 0.1f + 2.0f / (2 * _resolution);
		float thickness = longest_axis / _resolution;
		float offset = 2 * thickness + margin;
		float axis_len = longest_axis + 2 * offset;
		float cell_size = axis_len / _resolution;

		float new_xmin = bbox_origin.xmin() - offset;
		float new_ymin = bbox_origin.ymin() - offset;
		float new_zmin = bbox_origin.zmin() - offset;

		CGAL::Bbox_3 grid_aabb(
			new_xmin,
			new_ymin,
			new_zmin,
			new_xmin + axis_len,
			new_ymin + axis_len,
			new_zmin + axis_len
		);

		_bboxMin[0] = new_xmin, _bboxMin[1] = new_ymin, _bboxMin[2] = new_zmin;
		_cellSize = cell_size;
		ExactPoint grid_min(grid_aabb.xmin(), grid_aabb.ymin(), grid_aabb.zmin());

		// index where the actual data is finished at each axis
		int data_res[3] = {
			static_cast<int>(ceil((bbox_origin.xmax() - grid_aabb.xmin()) / cell_size)),
			static_cast<int>(ceil((bbox_origin.ymax() - grid_aabb.ymin()) / cell_size)),
			static_cast<int>(ceil((bbox_origin.zmax() - grid_aabb.zmin()) / cell_size))
		};
		
		std::array<unsigned int, 3> numVoxels = { _resolution, _resolution, _resolution };
		std::array<ExactVector, 3> voxel_strides = { ExactVector(cell_size, 0, 0),
			ExactVector(0, cell_size, 0), ExactVector(0, 0, cell_size) };

		std::vector<unsigned int> intersecting_voxels;
		auto numVoxel = pow(_resolution, 3);// numVoxels[0] * numVoxels[1] * numVoxels[2];

		VOXEL_GRID voxels_marking(numVoxel, false); // either outside 0, surface 1 or interior 2
		tree.accelerate_distance_queries();
		Point_inside inside_tester(tree);

#pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < (int)numVoxel; i++) {

			unsigned int x_idx, y_idx, z_idx;
			convert_voxel_idx_to_coords(i, numVoxels[0], x_idx, y_idx, z_idx);
			if (x_idx >= data_res[0] || y_idx >= data_res[1] || z_idx >= data_res[2]) continue;
			Exact_Polyhedron voxel = Exact_Polyhedron();
			bool new_scanline;

			calc_voxel_from_idx_tets(i, numVoxels, grid_min, voxel_strides, voxel, &new_scanline);

			//check if inside
			bool inside = true;

			for (auto vert : voxel.vertex_handles())
			{
				if (inside_tester(vert->point()) != CGAL::ON_BOUNDED_SIDE)
				{
					inside = false;
					break;
				}
			}
			CGAL::Iso_cuboid_3<Exact_Kernel> voxel_bbox = CGAL::bounding_box(voxel.points().begin(), voxel.points().end());
			bool intersects = tree.any_intersected_primitive(voxel_bbox).has_value();

			if (intersects || inside) {
#pragma omp critical
				{
					voxels_marking[i] = true;
				}
			}
		}

		return voxels_marking;
	}

public:

	float GetCellSize() { return _cellSize; }
	std::array<float, 3> GetBboxMin() { return _bboxMin; }
	
	VOXEL_GRID GenerateVoxelGrid(std::string filename)
	{
		if (!_window)
			_voxelGrid = VoxelizeCGAL(filename);
		else
			_voxelGrid = VoxelizeShader(filename);

		return _voxelGrid;
	}
};