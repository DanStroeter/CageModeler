#pragma once

#include <vector>
#include <map>

#include <Eigen/Geometry>

typedef struct {
	float minFactor, maxFactor;
	std::map<int, Eigen::Vector3d> translations_per_vertex;
} Parametrization;

std::vector<float> createRegularSampling(int numSamples, float minValue = 0.f, float maxValue = 1.f);
Parametrization readParams(std::string paramFile);
Parametrization fromDeformedCage(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed);
