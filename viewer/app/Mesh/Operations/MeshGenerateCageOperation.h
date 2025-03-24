#pragma once

#include<Mesh/Operations/MeshOperation.h>
#include<filesystem>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <iostream>
#include <Eigen/Core>


struct GenerateCageFromMeshOperationParams{

GenerateCageFromMeshOperationParams(std::filesystem::path meshFilepath,
                               std::filesystem::path cageFilepath, 
                               const int scale,
                               const int smoothIterations,
                               const int targetNumFaces,
                               std::vector<bool>& closingResult
                               ):

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


class GenerateCageFromMeshOperation final:public MeshOperationTemplated<GenerateCageFromMeshOperationParams,void>
{
    public:
    using MeshOperationTemplated::MeshOperationTemplated;
    [[nodiscard]] std::string GetDescription() const override
	{
		return "Generating cage";
	}

	void Execute();

};

namespace{

	GLuint loadShader(const std::string& path, GLenum shaderType) {
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
	GLuint genVGFProgram(const std::string& vertexPath,
		const std::string& geometryPath,
		const std::string& fragmentPath) {
		GLuint vertexShader = loadShader(vertexPath, GL_VERTEX_SHADER);
		GLuint geometryShader = loadShader(geometryPath, GL_GEOMETRY_SHADER);
		GLuint fragmentShader = loadShader(fragmentPath, GL_FRAGMENT_SHADER);

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

	//void calcNormals(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd& normals)
	//{
	//	normals.resize(CF.rows(), 3);

	//	for (int i = 0; i < CF.rows(); ++i)
	//	{
	//		Eigen::Vector3i index_vector = CF.row(i);

	//		const Eigen::Vector3d t_0 = C.row(index_vector[0]);
	//		const Eigen::Vector3d t_1 = C.row(index_vector[1]);
	//		const Eigen::Vector3d t_2 = C.row(index_vector[2]);

	//		auto const normal = ((t_1 - t_0).cross(t_2 - t_0)).normalized();

	//		normals.row(i) = normal;
	//	}
	//}
}