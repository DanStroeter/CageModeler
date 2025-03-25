#pragma once

#include<Mesh/Operations/MeshOperation.h>
#include<filesystem>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <iostream>
#include <Eigen/Core>

#define BUFFER_OFFSET(offset) ((GLvoid*)(offset))

struct GenerateCageFromMeshOperationParams {

	GenerateCageFromMeshOperationParams(std::filesystem::path meshFilepath,
		std::filesystem::path cageFilepath,
		const int scale,
		const int smoothIterations,
		const int targetNumFaces,
		std::vector<bool>& closingResult
	) :

		_meshfilepath(std::move(meshFilepath)),
		_cagefilepath(std::move(cageFilepath)),
		_scale(scale),
		_smoothIterations(smoothIterations),
		_targetNumFaces(targetNumFaces),
		_closingResult(closingResult)
	{}

	std::filesystem::path _meshfilepath;
	std::filesystem::path _cagefilepath;
	int _scale;
	int _smoothIterations;
	int _targetNumFaces;
	std::vector<bool>& _closingResult;


};


class GenerateCageFromMeshOperation final :public MeshOperationTemplated<GenerateCageFromMeshOperationParams, void>
{
public:
	using MeshOperationTemplated::MeshOperationTemplated;
	[[nodiscard]] std::string GetDescription() const override
	{
		return "Generating cage";
	}

	void Execute();

};

class Voxelizer {

public:
	Voxelizer(int resolution) : _resolution(resolution) {}
	~Voxelizer() {};

private:
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
	float _cellSize = 0.f;


	std::vector<float> _vertices;
	std::vector<float> _normals;
	std::vector<unsigned int> _indices;
	std::vector<bool> _voxelGrid;

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

		std::cout << "Starting Surface Voxelization!!\n";

		GLuint query;
		GLuint64 elapsed_time;
		glGenQueries(1, &query);

		// 1) Use the Volume Voxelization Shader
		glUseProgram(program);
		printf("Using Conservative Surface Voxelization Program");


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
				std::cout << "enabling GL_CONSERVATIVE_RASTERIZATION_NV\n";
			}
			else {
				glDisable(GL_CONSERVATIVE_RASTERIZATION_NV);
				std::cout << "disabling GL_CONSERVATIVE_RASTERIZATION_NV\n";
			}
		}
		else if (glewIsSupported("GL_INTEL_conservative_rasterization")) {
			if (is_surface) {
				glEnable(GL_CONSERVATIVE_RASTERIZATION_INTEL);
				std::cout << "enabling GL_CONSERVATIVE_RASTERIZATION_INTEL\n";
			}
			else {
				glDisable(GL_CONSERVATIVE_RASTERIZATION_INTEL);
				std::cout << "disabling GL_CONSERVATIVE_RASTERIZATION_INTEL\n";
			}

		}
		else {
			std::cerr << "[WARNING] GL_NV_conservative_raster/GL_INTEL_conservative_rasterization is not supported on this GPU." << std::endl;
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

		if (is_surface) {
			std::cout << "[Voxelizer] : " << "conservative surface voxelization done in "
				<< elapsed_time / 1000000.0 << " ms." << std::endl;
		}
		else {
			std::cout << "[Voxelizer] : " << "conservative volume voxelization done in "
				<< elapsed_time / 1000000.0 << " ms." << std::endl;
		}
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
		std::vector<bool>& unpack_vox_grid) {
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


public:
	float GetCellSize() { return _cellSize; }
	std::array<float, 3> GetBboxMin() { return _bboxMin; }
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

	std::vector<bool> GenerateVoxelGrid(Eigen::MatrixXd& V_model, Eigen::MatrixXd& N_model, Eigen::MatrixXi& T_model) {
		GLFWwindow* window = initOpenGL();
		
		if (!window) return _voxelGrid; // return empty vector

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
		float se_size = 0.1f;
		float margin = se_size + 2.0f / (2 * _resolution);
		float longest_axis = std::max(_bboxMax[0] - _bboxMin[0],
			std::max(_bboxMax[1] - _bboxMin[1], _bboxMax[2] - _bboxMin[2]));
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
		int res = 2 * base_res_; // the base_res_ is the resolution of the packed layout
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

		glfwDestroyWindow(window);
		glfwTerminate();

		return _voxelGrid;

	}
};